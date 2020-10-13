import * as React from "react";
import styles from "./PanelSidebar.module.scss";
import { Form } from "react-bootstrap";
import { useStores } from "../../hooks/useStores";
import { useObserver } from "mobx-react-lite";
import { ChangeEvent, useEffect } from "react";
import { EventTypeId, eventTypeIds } from "../../registry/events";
import { Panel } from "../../stores/VisualizationStore";
import { autorun } from "mobx";

interface IProps {
  panel: Panel;
}

export const PanelSidebar: React.FC<IProps> = ({ panel }) => {
  const { visualizationStore: store } = useStores();

  // Automatically choose an event type when a buffer is loaded
  useEffect(
    () =>
      autorun(() => {
        if (!panel.eventType && !store.bufferEmpty) {
          panel.setEventType(Object.keys(store.buffer)[0] as EventTypeId);
        }
      }),
    []
  );

  return useObserver(() => {
    const {
      eventType,
      tagFilter,
      selectedVisualizer,
      visualizers,
      optionConfigs,
      selectedOptions
    } = panel;
    const { bufferEmpty } = store;

    return (
      <div className={styles.panelSidebar}>
        <Form.Group controlId="eventType">
          <Form.Label>Data Type</Form.Label>
          <Form.Control
            as="select"
            disabled={bufferEmpty}
            value={!bufferEmpty && eventType ? eventType : ""}
            onChange={(e: ChangeEvent<HTMLSelectElement>): void => {
              if (e.target.value) {
                panel.setEventType(e.target.value as EventTypeId);
              }
            }}
          >
            {[...eventTypeIds].map((_) => (
              <option disabled={!(_ in store.buffer)} value={_} key={_}>
                {_.split(".").slice(-1)}
              </option>
            ))}
          </Form.Control>
        </Form.Group>

        <Form.Group controlId="tags">
          <Form.Label>Tag Filter</Form.Label>
          <Form.Control
            value={tagFilter}
            onChange={(e: ChangeEvent<HTMLInputElement>) => {
              panel.tagFilter = e.target.value;
            }}
            disabled={bufferEmpty}
          />
          <Form.Text className="text-muted">
            Supports regular expressions
          </Form.Text>
        </Form.Group>

        <Form.Group controlId="visualizer">
          <Form.Label>Visualizer</Form.Label>
          <Form.Control
            as="select"
            value={selectedVisualizer}
            onChange={(e: ChangeEvent<HTMLSelectElement>) =>
              panel.setVisualizer(parseInt(e.target.value))
            }
            disabled={bufferEmpty}
          >
            {visualizers.map((v, index) => (
              <option key={v.id} value={index}>
                {v.id}
              </option>
            ))}
          </Form.Control>
        </Form.Group>

        {optionConfigs.map((optionLabel, optionIndex) => (
          <Form.Group
            key={optionLabel.label}
            controlId={`option.{option.label}`}
          >
            <Form.Label>{optionLabel.label}</Form.Label>
            <Form.Control
              as="select"
              value={selectedOptions[optionIndex]}
              onChange={(e: ChangeEvent<HTMLSelectElement>) =>
                panel.setOption(optionIndex, parseInt(e.target.value))
              }
              disabled={bufferEmpty}
            >
              {optionLabel.options.map((valueLabel, valueIndex) => (
                <option key={valueLabel} value={valueIndex}>
                  {valueLabel}
                </option>
              ))}
            </Form.Control>
          </Form.Group>
        ))}
      </div>
    );
  });
};
