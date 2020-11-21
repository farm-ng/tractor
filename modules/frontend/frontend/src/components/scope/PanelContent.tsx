import * as React from "react";
import styles from "./PanelContent.module.scss";
import { Stream } from "./Stream";
import { useStores } from "../../hooks/useStores";
import { useObserver } from "mobx-react-lite";
import { Panel } from "../../stores/VisualizationStore";

interface IProps {
  panel: Panel;
}

export const PanelContent: React.FC<IProps> = ({ panel }) => {
  const { visualizationStore: store } = useStores();

  return useObserver(() => {
    const { eventType } = panel;
    if (!eventType) {
      return null;
    }
    const eventTypeData = store.buffer[eventType];
    if (!eventTypeData) {
      return null;
    }
    const filter = new RegExp(panel.tagFilter);
    const streams = Object.entries(eventTypeData)
      .filter(([name, _]) => filter.test(name))
      .map(([name, values]) => (
        <Stream key={name} panel={panel} name={name} values={values} />
      ));
    return <div className={styles.panelContent}>{streams}</div>;
  });
};
