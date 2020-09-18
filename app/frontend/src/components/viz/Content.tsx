import * as React from "react";
import styles from "./Content.module.scss";
import { Panel } from "./Panel";
import { Button } from "react-bootstrap";
import { useStores } from "../../hooks/useStores";
import { useObserver } from "mobx-react-lite";
import { LogChooser } from "./LogChooser";
export const Content: React.FC = () => {
  const { visualizationStore: store } = useStores();

  return useObserver(() => {
    if (store.dataSource === "log" && store.bufferLoadProgress < 1) {
      return (
        <div className={styles.content}>
          <LogChooser />
        </div>
      );
    }
    const panels = Object.entries(
      Object.fromEntries(store.panels.entries())
    ).map(([_, p]) => <Panel key={p.id} id={p.id} />);

    return (
      <div className={styles.content}>
        {panels}
        <Button onClick={() => store.addPanel()}>+</Button>
      </div>
    );
  });
};