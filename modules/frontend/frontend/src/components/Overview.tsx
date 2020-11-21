import * as React from "react";
import { ListGroup, Card, CardColumns, Table } from "react-bootstrap";
import { useObserver } from "mobx-react-lite";
import { TractorState } from "@farm-ng/genproto-tractor/farm_ng/tractor/tractor";
import { MotorControllerState } from "@farm-ng/genproto-tractor/farm_ng/tractor/motor";
import { formatValue } from "../utils/formatValue";
import { useStores } from "../hooks/useStores";
import styles from "./Overview.module.scss";

const voltageWarningThreshold = 38; // v
const processWarningThreshold = 5000; // ms

export const Overview: React.FC = () => {
  const { busEventStore } = useStores();
  return useObserver(() => {
    const absDistanceTraveled = (busEventStore.lastSnapshot.get("tractor_state")
      ?.latestEvent as TractorState)?.absDistanceTraveled;

    const rightMotorInputVoltage = (busEventStore.lastSnapshot.get(
      "right_motor/state"
    )?.latestEvent as MotorControllerState)?.inputVoltage;
    const rightMotorWarning =
      rightMotorInputVoltage &&
      rightMotorInputVoltage < voltageWarningThreshold;

    const leftMotorInputVoltage = (busEventStore.lastSnapshot.get(
      "left_motor/state"
    )?.latestEvent as MotorControllerState)?.inputVoltage;
    const leftMotorWarning =
      leftMotorInputVoltage && leftMotorInputVoltage < voltageWarningThreshold;

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
              <Card.Title>Distance Traveled</Card.Title>
              <ListGroup>
                <ListGroup.Item>
                  {formatValue(absDistanceTraveled || 0)} m
                </ListGroup.Item>
              </ListGroup>
            </Card.Body>
          </Card>

          <Card bg={"light"} className={"shadow-sm"}>
            <Card.Body>
              <Card.Title>Battery</Card.Title>
              <ListGroup horizontal>
                <ListGroup.Item className="flex-fill">
                  {leftMotorWarning && "⚠️"}
                  {leftMotorInputVoltage && `${leftMotorInputVoltage} v`}
                </ListGroup.Item>
                <ListGroup.Item className="flex-fill">
                  {rightMotorWarning && "⚠️"}
                  {rightMotorInputVoltage && `${rightMotorInputVoltage} v`}
                </ListGroup.Item>
              </ListGroup>
            </Card.Body>
          </Card>

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
