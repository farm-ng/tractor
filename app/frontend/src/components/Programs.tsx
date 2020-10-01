import * as React from "react";
import { Button, Table } from "react-bootstrap";
import { useStores } from "../hooks/useStores";
import {
  StartProgramRequest,
  StopProgramRequest
} from "../../genproto/farm_ng_proto/tractor/v1/program_supervisor";
import { useObserver } from "mobx-react-lite";
import { useEffect } from "react";
import { formatValue } from "../utils/formatValue";
import { RigCalibration } from "./RigCalibration";
import styles from "./Programs.module.scss";

export const Programs: React.FC = () => {
  const { programsStore: store } = useStores();

  useEffect(() => {
    store.startStreaming();
    return () => store.stopStreaming();
  }, []);

  const handleStart = (id: number): void => {
    store.busClient.send(
      "type.googleapis.com/farm_ng_proto.tractor.v1.StartProgramRequest",
      "program_supervisor/request",
      StartProgramRequest.encode(StartProgramRequest.fromJSON({ id })).finish()
    );
  };

  const handleStop = (id: number): void => {
    store.busClient.send(
      "type.googleapis.com/farm_ng_proto.tractor.v1.StopProgramRequest",
      "program_supervisor/request",
      StopProgramRequest.encode(StopProgramRequest.fromJSON({ id })).finish()
    );
  };

  return useObserver(() => {
    const rows = store.programSupervisorStatus?.library.map(
      ({ id, name, description }) => (
        <tr key={id}>
          <td>{name}</td>
          <td>{description}</td>
          <td>
            {store.runningProgram?.id === id
              ? "Active"
              : store.lastProgram?.id === id
              ? "Complete"
              : null}
          </td>
          <td>
            {store.runningProgram?.id === id
              ? store.runningProgram?.pid
              : store.lastProgram?.id === id
              ? store.lastProgram?.pid
              : null}
          </td>
          <td>
            {store.lastProgram?.id === id ? store.lastProgram?.exitCode : null}
          </td>
          <td>
            {store.runningProgram?.id === id
              ? formatValue(store.runningProgram?.stampStart)
              : store.lastProgram?.id === id
              ? formatValue(store.lastProgram?.stampStart)
              : null}
          </td>
          <td>
            {store.lastProgram?.id === id
              ? formatValue(store.lastProgram?.stampEnd)
              : null}
          </td>
          <td>
            <Button
              disabled={store.runningProgram}
              onClick={() => handleStart(id)}
            >
              Start
            </Button>
          </td>
          <td>
            <Button
              disabled={store.runningProgram?.id !== id}
              onClick={() => handleStop(id)}
            >
              Stop
            </Button>
          </td>
        </tr>
      )
    );
    return (
      <div className={styles.programList}>
        <Table striped bordered size="sm" responsive="md">
          <thead>
            <tr>
              <th>Name</th>
              <th>Description</th>
              <th>Status</th>
              <th>PID</th>
              <th>Exit Code</th>
              <th>Start</th>
              <th>End</th>
              <th></th>
              <th></th>
            </tr>
          </thead>
          <tbody>{rows}</tbody>
        </Table>
        {(store.runningProgram?.id === 1000 ||
          store.lastProgram?.id === 1000) && <RigCalibration />}
      </div>
    );
  });
};
