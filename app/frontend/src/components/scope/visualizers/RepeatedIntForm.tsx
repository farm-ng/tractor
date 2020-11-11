import * as React from "react";
import { useState } from "react";
import { Button, Table } from "react-bootstrap";
import { useFormState } from "../../../hooks/useFormState";
import { range } from "../../../utils/range";
import { uniquify } from "../../../utils/uniquify";
import Form from "./Form";

interface IProps {
  initialValue: number[];
  onChange: (updated: number[]) => void;
}

export const RepeatedIntForm: React.FC<IProps> = (props) => {
  const [value, setValue] = useFormState(props);
  const [isAddingElements, setIsAddingElements] = useState(false);
  const [newRange, setNewRange] = useState({ start: 0, end: 0 });

  return (
    <>
      <Table striped bordered size="sm" responsive="md">
        {value.map((element, index) => (
          <tr key={index}>
            <td>{element}</td>
            <td>
              <Button
                onClick={() =>
                  setValue((v) => [...v.slice(0, index), ...v.slice(index + 1)])
                }
              >
                X
              </Button>
            </td>
          </tr>
        ))}
      </Table>

      {!isAddingElements && (
        <Form.ButtonGroup
          buttonText="+"
          onClick={() => setIsAddingElements(true)}
        />
      )}

      {isAddingElements && (
        <>
          <Form.Group
            label={`Range Start`}
            value={newRange.start}
            type="number"
            onChange={(e) => {
              const start = parseInt(e.target.value);
              setNewRange((r) => ({
                ...r,
                start
              }));
            }}
          />

          <Form.Group
            label={`Range End (inclusive)`}
            value={newRange.end}
            type="number"
            onChange={(e) => {
              const end = parseInt(e.target.value);
              setNewRange((r) => ({
                ...r,
                end
              }));
            }}
          />

          <Form.ButtonGroup
            buttonText="✓"
            onClick={() => {
              setValue((v) =>
                uniquify(
                  [...v, ...range(newRange.start, newRange.end + 1)].sort(
                    (a, b) => a - b
                  )
                )
              );
              setIsAddingElements(false);
            }}
          />

          <Form.ButtonGroup
            buttonText="×"
            onClick={() => setIsAddingElements(false)}
          />
        </>
      )}
    </>
  );
};
