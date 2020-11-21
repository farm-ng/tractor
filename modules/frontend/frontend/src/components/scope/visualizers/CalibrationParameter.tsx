import * as React from "react";
import { CalibrationParameter } from "@farm-ng/genproto-calibration/farm_ng/calibration/calibrator";
import { useFormState } from "../../../hooks/useFormState";
import { FormProps } from "../../../registry/visualization";
import Form from "./Form";
import { StandardComponentOptions } from "./StandardComponent";

interface IFormProps extends FormProps<CalibrationParameter> {
  valueLabel?: string;
}

const CalibrationParameterForm: React.FC<IFormProps> = ({
  valueLabel,
  ...props
}) => {
  const [value, setValue] = useFormState(props);
  return (
    <>
      <Form.Group
        label={valueLabel || "Value"}
        value={value.value}
        type="number"
        onChange={(e) => {
          const value = parseFloat(e.target.value);
          setValue((v) => ({ ...v, value }));
        }}
      />
      <Form.Group
        label="Constant"
        checked={value.constant}
        type="checkbox"
        onChange={(e: React.ChangeEvent<HTMLInputElement>) => {
          const constant = Boolean(e.target.checked);
          setValue((v) => ({ ...v, constant }));
        }}
      />
    </>
  );
};

export const CalibrationParameterVisualizer = {
  id: "CalibrationParameter",
  types: ["type.googleapis.com/farm_ng.calibration.CalibrationParameter"],
  options: StandardComponentOptions,
  Form: CalibrationParameterForm,
};
