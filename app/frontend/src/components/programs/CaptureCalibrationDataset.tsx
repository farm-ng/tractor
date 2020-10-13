/* eslint-disable no-console */
import * as React from "react";

import { useStores } from "../../hooks/useStores";
import { useState } from "react";
import { Event as BusEvent } from "../../../genproto/farm_ng_proto/tractor/v1/io";
import {
  CaptureCalibrationDatasetConfiguration,
  CaptureCalibrationDatasetConfiguration as Configuration,
  CaptureCalibrationDatasetStatus as Status
} from "../../../genproto/farm_ng_proto/tractor/v1/capture_calibration_dataset";
import { CaptureCalibrationDatasetConfigurationVisualizer } from "../scope/visualizers/CaptureCalibrationDatasetConfiguration";
import { ProgramProps } from "../../registry/programs";
import { decodeAnyEvent } from "../../models/decodeAnyEvent";
import Form from "../scope/visualizers/Form";

const programId = "capture_calibration_dataset";

const Component: React.FC<ProgramProps<Configuration>> = ({
  inputRequired
}) => {
  const { busClient } = useStores();
  const [configuration, setConfiguration] = useState<Configuration>();

  const handleConfigurationSubmit = (
    e: React.FormEvent<HTMLFormElement>
  ): void => {
    e.preventDefault();
    busClient.send(
      "type.googleapis.com/farm_ng_proto.tractor.v1.CaptureCalibrationDatasetConfiguration",
      `${programId}/configure`,
      Configuration.encode(Configuration.fromJSON(configuration)).finish()
    );
  };

  if (inputRequired && !configuration) {
    setConfiguration(inputRequired);
  }

  return (
    <Form onSubmit={handleConfigurationSubmit}>
      <CaptureCalibrationDatasetConfigurationVisualizer.Form
        initialValue={
          configuration ||
          CaptureCalibrationDatasetConfiguration.fromPartial({})
        }
        onChange={(updated) => setConfiguration(updated)}
      />
      <Form.ButtonGroup type="submit" buttonText="Submit" />
    </Form>
  );
};

export const CaptureCalibrationDatasetProgram = {
  programIds: [programId] as const,
  eventLogPredicate: (e: BusEvent) =>
    e.name.startsWith(`${programId}/`) || e.name.startsWith("calibrator/"),
  inputRequired: (e: BusEvent) => {
    if (!e.name.startsWith(`${programId}/status`)) {
      return null;
    }
    const data = decodeAnyEvent(e);
    if (!data) {
      console.error(`Could not decode bus event`, e);
      return null;
    }
    return (data as Status).inputRequiredConfiguration || null;
  },
  Component
};
