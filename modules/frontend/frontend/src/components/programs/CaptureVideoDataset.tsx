/* eslint-disable no-console */
import * as React from "react";
import { useStores } from "../../hooks/useStores";
import { Event as BusEvent } from "@farm-ng/genproto-core/farm_ng/core/io";
import { useState } from "react";
import Form from "../scope/visualizers/Form";
import {
  CaptureVideoDatasetConfiguration,
  CaptureVideoDatasetConfiguration as Configuration,
  CaptureVideoDatasetStatus as Status,
} from "@farm-ng/genproto-perception/farm_ng/perception/capture_video_dataset";
import { CaptureVideoDatasetConfigurationVisualizer } from "../scope/visualizers/CaptureVideoDatasetConfiguration";
import { ProgramProps } from "../../registry/programs";
import { decodeAnyEvent } from "../../models/decodeAnyEvent";

const programId = "capture_video_dataset";

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
      <CaptureVideoDatasetConfigurationVisualizer.Form
        initialValue={
          configuration || CaptureVideoDatasetConfiguration.fromPartial({})
        }
        onChange={(updated) => setConfiguration(updated)}
      />
      <Form.ButtonGroup type="submit" buttonText="Submit" />
    </Form>
  );
};

export const CaptureVideoDatasetProgram = {
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
