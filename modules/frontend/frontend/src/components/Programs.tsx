import * as React from "react";
import { Alert, Button, Collapse, Table } from "react-bootstrap";
import { useStores } from "../hooks/useStores";
import {
  StartProgramRequest,
  StopProgramRequest,
} from "@farm-ng/genproto-core/farm_ng/core/programd";
import { useObserver } from "mobx-react-lite";
import { useEffect, useMemo } from "react";
import { formatValue } from "../utils/formatValue";
import styles from "./Programs.module.scss";
import { EventVisualizer } from "./scope/visualizers/Event";

export const Programs: React.FC = () => {
  const { busClient, httpResourceArchive, programsStore: store } = useStores();

  useEffect(() => {
    store.startStreaming();
    return () => store.stopStreaming();
  }, []);

  const handleStart = (id: string): void => {
    busClient.send(
      "type.googleapis.com/farm_ng.core.StartProgramRequest",
      "programd/request",
      StartProgramRequest.encode(StartProgramRequest.fromJSON({ id })).finish()
    );
  };

  const handleStop = (id: string): void => {
    busClient.send(
      "type.googleapis.com/farm_ng.core.StopProgramRequest",
      "programd/request",
      StopProgramRequest.encode(StopProgramRequest.fromJSON({ id })).finish()
    );
  };

  return useObserver(() => {
    const rows = store.supervisorStatus?.library.map(
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

    // Explicitly recreate inputRequiredElement when its inputs change,
    // otherwise, `React.createElement` will be invoked on every re-render of this component.
    const inputRequiredElement = useMemo(
      () =>
        store.program &&
        store.inputRequired &&
        React.createElement(store.program.Component, {
          inputRequired: store.inputRequired,
        }),
      [store.program, store.inputRequired]
    );

    return (
      <div className={styles.programs}>
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
        <div className={styles.programDetail}>
          <Collapse in={Boolean(store.inputRequired)}>
            <div>
              <div className={styles.programForm}>
                <Alert variant="warning">User Input Requested</Alert>
                {inputRequiredElement}
              </div>
            </div>
          </Collapse>
          <EventVisualizer.Component
            values={store.eventLog}
            options={[{ label: "", options: [], value: "overlay" }]}
            resources={httpResourceArchive}
          />
        </div>
      </div>
    );
  });
};
