/* eslint-disable no-console */
import * as React from "react";
import { useStores } from "../../hooks/useStores";
import { Event as BusEvent } from "@farm-ng/genproto-core/farm_ng/core/io";
import { useState } from "react";
import Form from "../scope/visualizers/Form";
import {
  CalibrateMultiViewApriltagRigConfiguration as Configuration,
  CalibrateMultiViewApriltagRigStatus as Status,
} from "@farm-ng/genproto-calibration/farm_ng/calibration/calibrate_multi_view_apriltag_rig";
import { ProgramProps } from "../../registry/programs";
import { decodeAnyEvent } from "../../models/decodeAnyEvent";
import { CalibrateMultiViewApriltagRigConfigurationVisualizer } from "../scope/visualizers/CalibrateMultiViewApriltagRigConfiguration";

const programId = "calibrate_multi_view_apriltag_rig";

const Component: React.FC<ProgramProps<Configuration>> = ({
  inputRequired,
}) => {
  const { busClient } = useStores();
  const [configuration, setConfiguration] = useState<Configuration | null>(
    null
  );

  const handleConfigurationSubmit = (
    e: React.FormEvent<HTMLFormElement>
  ): void => {
    e.preventDefault();
    busClient.send(
      "type.googleapis.com/farm_ng.calibration.CalibrateMultiViewApriltagRigConfiguration",
      `${programId}/configure`,
      Configuration.encode(Configuration.fromJSON(configuration)).finish()
    );
  };

  if (inputRequired && !configuration) {
    setConfiguration(inputRequired);
  }

  return (
    <Form onSubmit={handleConfigurationSubmit}>
      <CalibrateMultiViewApriltagRigConfigurationVisualizer.Form
        initialValue={configuration || Configuration.fromPartial({})}
        onChange={(updated) => setConfiguration(updated)}
      />
      <Form.ButtonGroup type="submit" buttonText="Submit" />
    </Form>
  );
};

export const CalibrateMultiViewApriltagRigProgram = {
  programIds: [programId] as const,
  // TODO(ethanrublee): Modify this as necessary
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
    return (data as Status).inputRequiredConfiguration || null;
  },
  Component,
};
