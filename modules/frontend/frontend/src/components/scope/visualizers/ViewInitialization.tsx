import * as React from "react";
import {
  CalibrationParameter,
  ViewDirection,
  viewDirectionToJSON,
  ViewInitialization,
} from "@farm-ng/genproto-calibration/farm_ng/calibration/calibrator";
import { useFormState } from "../../../hooks/useFormState";
import {
  FormProps,
  SingleElementVisualizerProps,
} from "../../../registry/visualization";
import { enumNumericKeys } from "../../../utils/enum";
import { CalibrationParameterVisualizer } from "./CalibrationParameter";
import { CalibrationParameterTable } from "./CalibrationParameterTable";
import Form from "./Form";
import { KeyValueTable } from "./KeyValueTable";
import {
  StandardComponentOptions,
  StandardComponent,
} from "./StandardComponent";

const ViewInitializationForm: React.FC<FormProps<ViewInitialization>> = (
  props
) => {
  const [value, setValue] = useFormState(props);
  return (
    <>
      <CalibrationParameterVisualizer.Form
        valueLabel="X"
        initialValue={value.x || CalibrationParameter.fromPartial({})}
        onChange={(updated) => setValue((v) => ({ ...v, x: updated }))}
      />
      <CalibrationParameterVisualizer.Form
        valueLabel="Y"
        initialValue={value.y || CalibrationParameter.fromPartial({})}
        onChange={(updated) => setValue((v) => ({ ...v, y: updated }))}
      />
      <CalibrationParameterVisualizer.Form
        valueLabel="Z"
        initialValue={value.z || CalibrationParameter.fromPartial({})}
        onChange={(updated) => setValue((v) => ({ ...v, z: updated }))}
      />

      <Form.Group
        label="View Direction"
        value={value.viewDirection}
        as="select"
        onChange={(e) => {
          const viewDirection = parseInt(e.target.value);
          setValue((v) => ({ ...v, viewDirection }));
        }}
      >
        {enumNumericKeys(ViewDirection)
          .filter((k) => k >= 0)
          .map((k) => {
            return (
              <option key={k} value={k}>
                {viewDirectionToJSON(k)}
              </option>
            );
          })}
      </Form.Group>
    </>
  );
};

const ViewInitializationElement: React.FC<SingleElementVisualizerProps<
  ViewInitialization
>> = ({ value: [, value] }) => {
  const { x, y, z } = value;
  return (
    <>
      {x && y && z && (
        <CalibrationParameterTable
          labels={["x", "y", "z"]}
          parameters={[x, y, z]}
        />
      )}
      <KeyValueTable
        records={[["Direction", viewDirectionToJSON(value.viewDirection)]]}
      />
    </>
  );
};

export const ViewInitializationVisualizer = {
  id: "ViewInitialization",
  types: ["type.googleapis.com/farm_ng.calibration.ViewInitialization"],
  options: StandardComponentOptions,
  Component: StandardComponent(ViewInitializationElement),
  Element: ViewInitializationElement,
  Form: ViewInitializationForm,
};
