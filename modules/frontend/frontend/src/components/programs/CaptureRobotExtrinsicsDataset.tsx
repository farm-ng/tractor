/* eslint-disable no-console */
import * as React from "react";
import { useStores } from "../../hooks/useStores";
import { Event as BusEvent } from "@farm-ng/genproto-core/farm_ng/core/io";
import { useState } from "react";
import Form from "../scope/visualizers/Form";
import { CaptureRobotExtrinsicsDatasetStatus as Status } from "@farm-ng/genproto-calibration/farm_ng/calibration/capture_robot_extrinsics_dataset";
import { ProgramProps } from "../../registry/programs";
import { decodeAnyEvent } from "../../models/decodeAnyEvent";
import { ResourceVisualizer } from "../scope/visualizers/Resource";
import { Resource } from "@farm-ng/genproto-core/farm_ng/core/resource";

const programId = "capture_robot_extrinsics_dataset";

const MultiElement: React.FC<ProgramProps<Resource>> = ({ inputRequired }) => {
  const { busClient } = useStores();
  const [resource, setResource] = useState<Resource>();

  const handleConfigurationSubmit = (
    e: React.FormEvent<HTMLFormElement>
  ): void => {
    e.preventDefault();
    if (resource === undefined) {
      console.error("Could not submit undefined configuration.");
      return;
    }
    busClient.send(resource, `${programId}/configure`, Resource);
  };

  if (inputRequired && !resource) {
    setResource(inputRequired);
  }

  return (
    <Form onSubmit={handleConfigurationSubmit}>
      <ResourceVisualizer.Form
        initialValue={resource || Resource.fromPartial({})}
        onChange={(updated) => setResource(updated)}
      />
      <Form.ButtonGroup type="submit" buttonText="Submit" />
    </Form>
  );
};

export const CaptureRobotExtrinsicsDatasetProgram = {
  programIds: [programId] as const,
  eventLogPredicate: (e: BusEvent) => e.name.startsWith(`${programId}/`),
  inputRequired: (e: BusEvent) => {
    if (!e.name.startsWith(`${programId}/status`)) {
      return null;
    }
    const data = decodeAnyEvent(e);
    if (!data) {
      console.error(`Could not decode bus event`, e);
      return null;
    }
    return (data as Status).inputRequiredResource || null;
  },
  MultiElement,
};
