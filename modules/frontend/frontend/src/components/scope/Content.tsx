import * as React from "react";
import styles from "./Content.module.scss";
import { Panel } from "./Panel";
import { useStores } from "../../hooks/useStores";
import { useObserver } from "mobx-react-lite";
import { Button } from "react-bootstrap";
import { Icon } from "../Icon";
export const Content: React.FC = () => {
  const { visualizationStore: store } = useStores();

  return useObserver(() => {
    if (store.bufferEmpty) {
      return null;
    }
    const panels = Object.values(store.panels).map((panel) => (
      <Panel key={panel.id} panel={panel} />
    ));

    return (
      <div className={styles.content}>
        {panels}
        <Button
          className={styles.addButton}
          variant="light"
          onClick={() => store.addPanel()}
        >
          <Icon id="plus" />
        </Button>
      </div>
    );
  });
};
