import * as React from "react";
import { Table } from "react-bootstrap";
import { useObserver } from "mobx-react-lite";
import { flatten } from "../utils/flatten";
import { formatValue } from "../utils/formatValue";
import styles from "./State.module.scss";
import { useStores } from "../hooks/useStores";

export const State: React.FC = () => {
  const { busEventStore } = useStores();

  return useObserver(() => {
    const rows = Object.entries(
      Object.fromEntries(busEventStore.lastSnapshot.entries())
    ).map(([key, stream]) => (
      <React.Fragment key={key}>
        <tr className={styles.primaryKey}>
          <td>{key}</td>
          <td>{stream.latestEventTime?.toISOString()}</td>
          <td>{stream.eventsSinceLastSnapshot}</td>
          <td></td>
        </tr>
        {Object.entries(flatten(stream.latestEvent || {})).map(
          ([subKey, value]) => (
            <tr key={subKey}>
              <td className={styles.subKey}>{subKey}</td>
              <td></td>
              <td></td>
              <td>{formatValue(value)}</td>
            </tr>
          )
        )}
      </React.Fragment>
    ));

    return (
      <Table striped bordered hover size="sm" responsive="md">
        <thead>
          <tr>
            <th>Key</th>
            <th>Latest Timestamp</th>
            <th>Events/s</th>
            <th>Latest Value</th>
          </tr>
        </thead>
        <tbody>{rows}</tbody>
      </Table>
    );
  });
};
