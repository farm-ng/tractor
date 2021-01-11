/* eslint-disable no-console */
import * as React from "react";
import { useState } from "react";
import { useStores } from "../../hooks/useStores";
import { Event as BusEvent } from "@farm-ng/genproto-core/farm_ng/core/io";
import {
  CalibrateIntrinsicsConfiguration,
  CalibrateIntrinsicsConfiguration as Configuration,
  CalibrateIntrinsicsStatus as Status,
} from "@farm-ng/genproto-calibration/farm_ng/calibration/calibrate_intrinsics";
import { CalibrateIntrinsicsConfigurationVisualizer } from "../scope/visualizers/CalibrateIntrinsicsConfiguration";
import { ProgramProps } from "../../registry/programs";
import { decodeAnyEvent } from "../../models/decodeAnyEvent";
import Form from "../scope/visualizers/Form";

const programId = "calibrate_intrinsics";

const MultiElement: React.FC<ProgramProps<Configuration>> = ({
  inputRequired,
}) => {
  const { busClient } = useStores();
  const [configuration, setConfiguration] = useState<Configuration>();

  const handleConfigurationSubmit = (
    e: React.FormEvent<HTMLFormElement>
  ): void => {
    e.preventDefault();
    if (configuration === undefined) {
      console.error("Could not submit undefined configuration.");
      return;
    }
    busClient.send(configuration, `${programId}/configure`, Configuration);
  };

  if (inputRequired && !configuration) {
    setConfiguration(inputRequired);
  }

  return (
    <Form onSubmit={handleConfigurationSubmit}>
      <CalibrateIntrinsicsConfigurationVisualizer.Form
        initialValue={
          configuration || CalibrateIntrinsicsConfiguration.fromPartial({})
        }
        onChange={(updated) => setConfiguration(updated)}
      />
      <Form.ButtonGroup type="submit" buttonText="Submit" />
    </Form>
  );
};

export const CalibrateIntrinsicsProgram = {
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
    return (data as Status).inputRequiredConfiguration || null;
  },
  MultiElement,
};
