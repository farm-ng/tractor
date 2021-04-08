/* eslint-disable no-console */
import * as React from "react";
import {
  FormProps,
  SingleElementVisualizerProps,
  visualizersForEventType,
} from "../../../registry/visualization";
import {
  StandardMultiElementOptions,
  StandardMultiElement,
} from "./StandardMultiElement";
import { Card } from "./Card";
import { Resource } from "@farm-ng/genproto-perception/farm_ng/core/resource";
import { useFormState } from "../../../hooks/useFormState";
import Form from "./Form";
import { useFetchResource } from "../../../hooks/useFetchResource";
import { parseResourceContentType } from "../../../utils/protoConversions";
import styles from "./Resource.module.scss";
import { Button, Modal } from "react-bootstrap";
import { Icon } from "../../Icon";
import { useRef, useState } from "react";
import { BlobstoreBrowser } from "../../BlobstoreBrowser";

interface ResourceFormProps extends FormProps<Resource> {
  label?: string;
}

const ResourceForm: React.FC<ResourceFormProps> = (props) => {
  const [value, setValue] = useFormState(props);
  const [showModal, setShowModal] = useState(false);
  const fileSelected = useRef<string | null>(null);

  const handleShowModal = () => setShowModal(true);
  const handleFileSelected = (path: string) => {
    fileSelected.current = path;
  };
  const handleCancelModal = () => {
    fileSelected.current = null;
    setShowModal(false);
  };
  const handleSubmitModal = () => {
    const fileSelectedCurrent = fileSelected.current;
    if (fileSelectedCurrent) {
      setValue((v) => ({
        ...v,
        path: fileSelectedCurrent,
      }));
    }
    setShowModal(false);
  };

  const shortContentType = value.contentType.replace(
    /type=type\.googleapis\.com\/farm_ng\.\w+\./,
    ""
  );

  // A form,
  // with two text fields for path and content_type (eventually support data too)
  // and a button next to path that launches a modal, allowing for selecting of a path
  // modal can select, preview, confirm or cancel.
  return (
    <>
      <Form.Group
        label={props.label || "Resource Path"}
        value={value.path}
        type="text"
        description={shortContentType}
        append={
          <Button variant="outline-secondary" onClick={handleShowModal}>
            <Icon id="folder2-open" />
          </Button>
        }
        onChange={(e) => {
          const path = e.target.value;
          setValue((v) => ({
            ...v,
            path,
          }));
        }}
      />

      <Modal
        show={showModal}
        onHide={handleCancelModal}
        dialogClassName={styles.modal}
      >
        <Modal.Header closeButton>
          <Modal.Title>Select a file</Modal.Title>
        </Modal.Header>
        <Modal.Body>
          <BlobstoreBrowser
            onFileSelected={handleFileSelected}
            editingEnabled={false}
            openInNewTabEnabled={false}
            updateUrlEnabled={false}
          ></BlobstoreBrowser>
        </Modal.Body>
        <Modal.Footer>
          <Button variant="secondary" onClick={handleCancelModal}>
            Close
          </Button>
          <Button variant="primary" onClick={handleSubmitModal}>
            Select
          </Button>
        </Modal.Footer>
      </Modal>
    </>
  );
};

const ResourceElement: React.FC<SingleElementVisualizerProps<Resource>> = (
  props
) => {
  const {
    value: [timestamp, value],
    resources,
  } = props;

  const [, typeUrl] = parseResourceContentType(value);
  const visualizer = visualizersForEventType(typeUrl)[0];
  if (!visualizer || !visualizer.Element) {
    return null;
  }

  const data = useFetchResource(value, resources);
  if (!data) {
    return null;
  }

  return (
    <Card timestamp={timestamp} json={value}>
      {React.createElement(visualizer.Element, {
        ...props,
        value: [timestamp, data],
      })}
    </Card>
  );
};

export const ResourceVisualizer = {
  id: "Resource",
  types: ["type.googleapis.com/farm_ng.core.Resource"],
  options: StandardMultiElementOptions,
  MultiElement: StandardMultiElement(ResourceElement),
  Element: ResourceElement,
  Form: ResourceForm,
};
