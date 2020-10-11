/* eslint-disable no-console */
import {
  FileBrowser,
  FileSearch,
  FileToolbar,
  FileList,
  FileData,
  ChonkyActions,
  FileAction,
  FileActionData
} from "chonky";
import * as React from "react";
import "chonky/style/main.css";
import { useEffect, useState } from "react";
import { File } from "../../genproto/farm_ng_proto/tractor/v1/resource";
import styles from "./Blobstore.module.scss";
import { eventRegistry, EventType, EventTypeId } from "../registry/events";
import { visualizersForEventType } from "../registry/visualization";
import { Button } from "react-bootstrap";
import { useStores } from "../hooks/useStores";
import { useHistory, useParams } from "react-router-dom";

const fileToFileData = (f: File): FileData => ({
  id: f.name,
  name: f.name,
  isDir: Boolean(f.directory),
  modDate: f.modificationTime,
  size: parseInt((f.size as unknown) as string) // TODO: due to issues with ts-proto Long
});

const dirsToPath = (dirs: FileData[]): string =>
  dirs.map((_) => _.name).join("/");

const bestGuessEventType = (
  folderChain: FileData[]
): EventTypeId | undefined => {
  if (folderChain.map((_) => _.name).includes("apriltag_rig_models")) {
    return "type.googleapis.com/farm_ng_proto.tractor.v1.CalibrateApriltagRigResult";
  }
  if (folderChain.map((_) => _.name).includes("configurations")) {
    return "type.googleapis.com/farm_ng_proto.tractor.v1.TractorConfig";
  }
  if (folderChain.map((_) => _.name).includes("base_to_camera_models")) {
    return "type.googleapis.com/farm_ng_proto.tractor.v1.CalibrateBaseToCameraResult";
  }
  if (folderChain.map((_) => _.name).includes("calibration-datasets")) {
    return "type.googleapis.com/farm_ng_proto.tractor.v1.CaptureCalibrationDatasetResult";
  }
  return undefined;
};

export const Blobstore: React.FC = () => {
  const [rootDir, setRootDir] = useState<FileData>();
  const [parentDirs, setParentDirs] = useState<FileData[]>([]);
  const [currentDir, setCurrentDir] = useState<File>();
  const [selectedPath, setSelectedPath] = useState<string>();
  const [selectedEventType, setSelectedEventType] = useState<EventTypeId>();
  const [selectedResource, setSelectedResource] = useState<EventType>();
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
      isDir: true
    }));
    setParentDirs(dirs);
  }

  // Handle file selection
  useEffect(() => {
    const fetchSelected = async (): Promise<void> => {
      if (!selectedPath) {
        return;
      }

      const eventType = bestGuessEventType(parentDirs);

      if (!eventType) {
        setSelectedResource(undefined);
        window.open(`${blobstoreUrl}/${selectedPath}`);
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
  }, [selectedPath]);

  // Fetch root directory
  useEffect(() => {
    const fetchRootDir = async (): Promise<void> => {
      const response = await fetch(`${blobstoreUrl}/`, {
        method: "GET",
        headers: {
          "Content-Type": "application/protobuf"
        }
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
        headers: {
          "Content-Type": "application/protobuf"
        }
      });
      const file = File.decode(new Uint8Array(await response.arrayBuffer()));
      setCurrentDir(file);
    };
    fetchDir();
    history.push(`/blobs/${dirsToPath(parentDirs)}`);
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

  const files = currentDir?.directory?.files.map<FileData>(fileToFileData);
  const visualizer =
    selectedEventType && visualizersForEventType(selectedEventType)[0];

  return (
    <div className={styles.container}>
      <div className={styles.browser}>
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
      <div className={styles.detail}>
        {selectedResource && !isEditing && visualizer?.Form && (
          <Button onClick={() => setIsEditing(true)}>{"Edit"}</Button>
        )}
        {selectedResource && isEditing && (
          <Button onClick={() => setIsEditing(false)}>{"Cancel"}</Button>
        )}
        {selectedResource && isEditing && (
          <Button onClick={() => setIsEditing(false)}>{"Submit"}</Button>
        )}
        {!isEditing && selectedResource && (
          <>
            {visualizer?.Element &&
              React.createElement(visualizer.Element, {
                value: [0, selectedResource],
                resources: httpResourceArchive
              })}
          </>
        )}
        {isEditing && selectedResource && visualizer?.Form && (
          <>
            {React.createElement(visualizer.Form, {
              initialValue: selectedResource,
              onChange: (updated) => console.log("Updated: ", updated)
            })}
          </>
        )}
      </div>
    </div>
  );
};
