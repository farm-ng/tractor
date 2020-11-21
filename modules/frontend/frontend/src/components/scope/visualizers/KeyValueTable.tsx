import * as React from "react";
import { Table } from "react-bootstrap";
import { formatValue } from "../../../utils/formatValue";

interface IProps {
  headers?: string[];
  records: [string, unknown][];
}

export const KeyValueTable: React.FC<IProps> = ({ records, headers }) => {
  return (
    <Table bordered size="sm" responsive="md">
      {headers && (
        <thead>
          <tr>
            {headers.map((_) => (
              <th key={_}>{_}</th>
            ))}
          </tr>
        </thead>
      )}
      <tbody>
        {records.map(([key, value]) => (
          <tr key={key}>
            <td>{key}</td>
            <td>{formatValue(value)}</td>
          </tr>
        ))}
      </tbody>
    </Table>
  );
};
