/* eslint-disable no-console */
import * as React from "react";
import {
  FileBrowser,
  FileSearch,
  FileToolbar,
  FileList,
  FileData,
  ChonkyActions,
  FileAction,
  FileActionData,
} from "chonky";
import "chonky/style/main.css";
import { useEffect, useState } from "react";
import { File } from "@farm-ng/genproto-core/farm_ng/core/resource";
import styles from "./BlobstoreBrowser.module.scss";
import { eventRegistry, EventType, EventTypeId } from "../registry/events";
import { visualizersForEventType } from "../registry/visualization";
import { Button } from "react-bootstrap";
import { useStores } from "../hooks/useStores";
import { useHistory, useParams } from "react-router-dom";
import Form from "./scope/visualizers/Form";
import { Icon } from "./Icon";

const fileToFileData = (f: File): FileData => ({
  id: f.name,
  name: f.name,
  isDir: Boolean(f.directory),
  modDate: f.modificationTime,
  size: parseInt((f.size as unknown) as string), // TODO: due to issues with ts-proto Long
});

const dirsToPath = (dirs: FileData[]): string =>
  dirs.map((_) => _.name).join("/");

const bestGuessEventType = (
  folderChain: FileData[],
  selectedPath: string
): EventTypeId | undefined => {
  if (folderChain.map((_) => _.name).includes("apriltag_rig_models")) {
    return "type.googleapis.com/farm_ng.calibration.CalibrateMultiViewApriltagRigResult";
  }
  if (folderChain.map((_) => _.name).includes("configurations")) {
    if (selectedPath.endsWith("tractor.json")) {
      return "type.googleapis.com/farm_ng.tractor.TractorConfig";
    }
    if (selectedPath.endsWith("apriltag.json")) {
      return "type.googleapis.com/farm_ng.perception.ApriltagConfig";
    }
    if (selectedPath.endsWith("camera.json")) {
      return "type.googleapis.com/farm_ng.perception.CameraPipelineConfig";
    }
    if (selectedPath.endsWith("capture_robot_extrinsics_dataset.json")) {
      return "type.googleapis.com/farm_ng.calibration.CaptureRobotExtrinsicsDatasetConfiguration";
    }
  }
  if (folderChain.map((_) => _.name).includes("base_to_camera_models")) {
    return "type.googleapis.com/farm_ng.calibration.CalibrateBaseToCameraResult";
  }
  if (folderChain.map((_) => _.name).includes("calibration_boards")) {
    return "type.googleapis.com/farm_ng.perception.ApriltagRig";
  }
  if (folderChain.map((_) => _.name).includes("calibration-datasets")) {
    return "type.googleapis.com/farm_ng.perception.CaptureVideoDatasetResult";
  }
  if (folderChain.map((_) => _.name).includes("robot_extrinsics_datasets")) {
    return "type.googleapis.com/farm_ng.calibration.CaptureRobotExtrinsicsDatasetResult";
  }
  if (folderChain.map((_) => _.name).includes("video_datasets")) {
    return "type.googleapis.com/farm_ng.perception.CreateVideoDatasetResult";
  }
  if (folderChain.map((_) => _.name).includes("intrinsic_models")) {
    return "type.googleapis.com/farm_ng.calibration.CalibrateIntrinsicsResult";
  }
  return undefined;
};

interface IProps {
  onFileSelected?: (path: string) => void;
  editingEnabled?: boolean;
  openInNewTabEnabled?: boolean;
  updateUrlEnabled?: boolean;
}

export const BlobstoreBrowser: React.FC<IProps> = ({
  onFileSelected = () => {},
  editingEnabled = true,
  openInNewTabEnabled = true,
  updateUrlEnabled = true,
}) => {
  const [rootDir, setRootDir] = useState<FileData>();
  const [parentDirs, setParentDirs] = useState<FileData[]>([]);
  const [currentDir, setCurrentDir] = useState<File>();
  const [selectedPath, setSelectedPath] = useState<string>();
  const [selectedEventType, setSelectedEventType] = useState<EventTypeId>();
  const [selectedResource, setSelectedResource] = useState<EventType>();
  const [modifiedResource, setModifiedResource] = useState<EventType>();
  const [modificationInFlight, setModificationInFlight] = useState(false);
  const [fullWidthDetail, setFullWidthDetail] = useState(false);
  const [isEditing, setIsEditing] = useState(false);

  const { baseUrl, httpResourceArchive } = useStores();
  const blobstoreUrl = `${baseUrl}/blobstore`;
  const history = useHistory();

  // Handle page load with nested URL
  // TODO: Support back button
  const { blobPath } = useParams<{ blobPath: string }>();
  if (!rootDir && parentDirs.length === 0 && blobPath) {
    const dirs = blobPath.split("/").map((path) => ({
      id: path,
      name: path,
      isDir: true,
    }));
    setParentDirs(dirs);
  }

  // Handle file selection
  useEffect(() => {
    const fetchSelected = async (): Promise<void> => {
      if (!selectedPath) {
        return;
      }
      if (modificationInFlight) {
        return;
      }

      onFileSelected(selectedPath);

      const eventType = bestGuessEventType(parentDirs, selectedPath);

      if (!eventType) {
        setSelectedResource(undefined);
        if (openInNewTabEnabled) {
          window.open(`${blobstoreUrl}/${selectedPath}`);
        }
        return;
      }
      try {
        const json = await httpResourceArchive.getJson(selectedPath);
        setSelectedEventType(eventType);
        setSelectedResource(eventRegistry[eventType].fromJSON(json));
      } catch (e) {
        console.error(`Error loading resource ${selectedPath}: ${e}`);
      }
    };
    fetchSelected();
  }, [selectedPath, modificationInFlight]);

  // Fetch root directory
  useEffect(() => {
    const fetchRootDir = async (): Promise<void> => {
      const response = await fetch(`${blobstoreUrl}/`, {
        method: "GET",
        cache: "no-store",
        headers: {
          "Content-Type": "application/protobuf",
        },
      });
      const file = File.decode(new Uint8Array(await response.arrayBuffer()));
      setRootDir(fileToFileData(file));
    };
    fetchRootDir();
  }, []);

  // Fetch current directory
  useEffect(() => {
    const fetchDir = async (): Promise<void> => {
      const path = dirsToPath(parentDirs);
      const response = await fetch(`${blobstoreUrl}/${path}`, {
        method: "GET",
        cache: "no-store",
        headers: {
          "Content-Type": "application/protobuf",
        },
      });
      const file = File.decode(new Uint8Array(await response.arrayBuffer()));
      setCurrentDir(file);
    };
    fetchDir();
    if (updateUrlEnabled) {
      history.push(`/blobs/${dirsToPath(parentDirs)}`);
    }
  }, [parentDirs]);

  const handleFileAction = (action: FileAction, data: FileActionData): void => {
    // Triggered by double-click
    if (action.id === ChonkyActions.OpenFiles.id) {
      const target = data?.target;
      if (!target) {
        return;
      }
      if (!target.isDir) {
        setSelectedPath(`${dirsToPath(parentDirs)}/${target.name}`);
        return;
      }
      // To detect navigation upwards in the directory hierarchy, look for
      // a parentDir with the target ID.
      // TODO: Support nested directories that have the same name. One approach would be to
      // use the full directory path as the id rather than just the directory name.
      const index = parentDirs.findIndex((_) => _.id === target.id);
      if (target.id === rootDir?.id) {
        setParentDirs([]);
      } else if (index >= 0) {
        setParentDirs((prev) => prev.slice(0, index + 1));
      } else {
        setParentDirs((prev) => [...prev, target]);
      }
    }
  };

  const handleCancel = (): void => {
    setIsEditing(false);
  };

  const toggleFullWidthDetail = (): void => {
    setFullWidthDetail((prev) => !prev);
  };

  const handleSubmit = async (
    e: React.FormEvent<HTMLFormElement>
  ): Promise<void> => {
    e.preventDefault();
    if (!selectedPath) {
      return;
    }

    const eventType = bestGuessEventType(parentDirs, selectedPath);

    if (!eventType) {
      console.error(`Aborting submit. Unknown eventType for ${selectedPath}`);
      return;
    }

    if (!modifiedResource) {
      console.error(`Aborting submit. No changes made.`);
      return;
    }

    const Message = eventRegistry[eventType];
    const message = Message.toJSON(Message.fromPartial(modifiedResource));
    setModificationInFlight(true);
    await fetch(`${blobstoreUrl}/${selectedPath}`, {
      method: "POST",
      body: JSON.stringify(message, null, 2),
      headers: {
        "Content-Type": "application/json",
      },
    });
    setModificationInFlight(false);
    setIsEditing(false);
  };

  const files = currentDir?.directory?.files.map<FileData>(fileToFileData);
  const visualizer =
    selectedEventType && visualizersForEventType(selectedEventType)[0];

  return (
    <div className={styles.container}>
      <div
        className={[styles.browser, fullWidthDetail ? styles.hidden : ""].join(
          " "
        )}
      >
        <FileBrowser
          files={files || []}
          folderChain={[rootDir, ...parentDirs].filter((_) => _)}
          onFileAction={handleFileAction}
          clearSelectionOnOutsideClick={false}
          defaultFileViewActionId={ChonkyActions.EnableListView.id}
        >
          <FileToolbar />
          <FileSearch />
          <FileList />
        </FileBrowser>
      </div>
      <div
        className={[
          styles.detail,
          fullWidthDetail ? styles.fullWidth : "",
        ].join(" ")}
      >
        {selectedResource && (
          <Button
            className={styles.fullWidthDetailButton}
            onClick={toggleFullWidthDetail}
          >
            <Icon
              id={fullWidthDetail ? "arrowsAngleContract" : "arrowsAngleExpand"}
            />
          </Button>
        )}
        {selectedResource &&
          editingEnabled &&
          !isEditing &&
          visualizer?.Form && (
            <Button onClick={() => setIsEditing(true)}>{"Edit"}</Button>
          )}
        {selectedResource && isEditing && (
          <Button onClick={handleCancel}>{"Cancel"}</Button>
        )}
        {!isEditing && selectedResource && (
          <>
            {visualizer?.Element &&
              React.createElement(visualizer.Element, {
                value: [0, selectedResource],
                resources: httpResourceArchive,
              })}
          </>
        )}
        {isEditing && selectedResource && visualizer?.Form && (
          <Form onSubmit={handleSubmit} className={styles.form}>
            {React.createElement(visualizer.Form, {
              initialValue: selectedResource,
              onChange: (updated) => {
                setModifiedResource(updated);
              },
            })}
            <Form.ButtonGroup type="submit" buttonText="Submit" />
          </Form>
        )}
      </div>
    </div>
  );
};
