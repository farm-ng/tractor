import * as React from "react";
import { decodeAnyEvent } from "../../models/decodeAnyEvent";
import { EventType } from "../../registry/events";
import { Program, ProgramProps } from "../../registry/programs";
import { Event as BusEvent } from "@farm-ng/genproto-core/farm_ng/core/io";
import { useStores } from "../../hooks/useStores";
import { useState } from "react";
import { DeepPartial, Message } from "../../types/common";
import Form from "../scope/visualizers/Form";
import { FormProps } from "../../registry/visualization";

function makeMultiElement<T extends EventType>(
  programId: string,
  Configuration: Message<T>,
  VisualizerForm: React.FC<FormProps<T>>
): React.FC<ProgramProps<T>> {
  return ({ inputRequired }) => {
    const { busClient } = useStores();
    const [configuration, setConfiguration] = useState<T>();

    const handleConfigurationSubmit = (
      e: React.FormEvent<HTMLFormElement>
    ): void => {
      e.preventDefault();
      if (configuration === undefined) {
        console.error("Could not submit undefined configuration.");
        return;
      }
      busClient.send(
        configuration as DeepPartial<T>,
        `${programId}/configure`,
        Configuration
      );
    };

    if (inputRequired && !configuration) {
      setConfiguration(inputRequired);
    }

    return (
      <Form onSubmit={handleConfigurationSubmit}>
        <VisualizerForm
          initialValue={
            configuration || Configuration.fromPartial({} as DeepPartial<T>)
          }
          onChange={(updated) => setConfiguration(updated)}
        />
        <Form.ButtonGroup type="submit" buttonText="Submit" />
      </Form>
    );
  };
}

export function makeStandardProgram<T extends EventType = EventType>(
  programId: string,
  Configuration: Message<T>,
  VisualizerForm: React.FC<FormProps<T>>
): Program<T> {
  interface Status {
    inputRequiredConfiguration: T;
  }

  return {
    programId,
    inputRequired: (e: BusEvent) => {
      if (!e.name.startsWith(`${programId}/status`)) {
        return null;
      }
      const data = decodeAnyEvent(e);
      if (!data) {
        console.error(`Could not decode bus event`, e);
        return null;
      }
      return ((data as unknown) as Status).inputRequiredConfiguration || null;
    },
    MultiElement: makeMultiElement(programId, Configuration, VisualizerForm),
  };
}
