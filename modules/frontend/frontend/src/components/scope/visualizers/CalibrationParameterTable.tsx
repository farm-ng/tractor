import * as React from "react";
import { Table } from "react-bootstrap";
import { CalibrationParameter } from "@farm-ng/genproto-calibration/farm_ng/calibration/calibrator";
import { formatValue } from "../../../utils/formatValue";

interface IProps {
  labels: string[];
  parameters: CalibrationParameter[];
}

export const CalibrationParameterTable: React.FC<IProps> = ({
  labels,
  parameters,
}) => {
  return (
    <Table bordered size="sm" responsive="md">
      <thead>
        <tr>
          <th>Label</th>
          <th>Value</th>
          <th>Constant</th>
        </tr>
      </thead>
      <tbody>
        {parameters.map((p, i) => (
          <tr key={labels[i]}>
            <td>{labels[i]}</td>
            <td>{formatValue(p.value)}</td>
            <td>{formatValue(p.constant)}</td>
          </tr>
        ))}
      </tbody>
    </Table>
  );
};
