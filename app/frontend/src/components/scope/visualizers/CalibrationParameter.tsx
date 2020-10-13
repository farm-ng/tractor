import * as React from "react";
import { CalibrationParameter } from "../../../../genproto/farm_ng_proto/tractor/v1/calibrator";
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
        onChange={(e: React.ChangeEvent<HTMLInputElement>) =>
          setValue((v) => ({ ...v, constant: Boolean(e.target.checked) }))
        }
      />
    </>
  );
};

export const CalibrationParameterVisualizer = {
  id: "CalibrationParameter",
  types: ["type.googleapis.com/farm_ng_proto.tractor.v1.CalibrationParameter"],
  options: StandardComponentOptions,
  Form: CalibrationParameterForm
};
