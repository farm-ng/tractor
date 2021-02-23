import * as React from "react";
import { ListGroup, Card, CardColumns, Table } from "react-bootstrap";
import { useObserver } from "mobx-react-lite";
import { formatValue } from "../utils/formatValue";
import { useStores } from "../hooks/useStores";
import styles from "./Overview.module.scss";

const voltageWarningThreshold = 38; // v
const processWarningThreshold = 5000; // ms

export const Overview: React.FC = () => {
  const { busEventStore } = useStores();
  return useObserver(() => {
    const processes = Array.from(busEventStore.lastSnapshot.keys())
      .filter((key) => key.startsWith("ipc/announcement"))
      .map<[string, Date?]>((key) => [
        key.replace("ipc/announcement/", ""),
        busEventStore.lastSnapshot.get(key)?.latestEventTime,
      ]);
    const processStatus = (
      <Table className={styles.processTable} responsive>
        <tbody>
          {processes.map(([serviceName, lastSeen]) => (
            <tr key={serviceName}>
              <td>
                {!lastSeen ||
                new Date().getTime() - lastSeen.getTime() >
                  processWarningThreshold
                  ? "⚠️"
                  : "✔️"}
              </td>
              <td>{serviceName}</td>
              <td>{formatValue(lastSeen)}</td>
            </tr>
          ))}
        </tbody>
      </Table>
    );

    return (
      <div className={styles.content}>
        <CardColumns>
            <Card bg={"light"} className={"shadow-sm"}>
            <Card.Body>
              <Card.Title>Processes</Card.Title>
              {processStatus}
            </Card.Body>
          </Card>
        </CardColumns>
      </div>
    );
  });
};
