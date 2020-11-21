import * as React from "react";
import styles from "./Panel.module.scss";
import { PanelSidebar } from "./PanelSidebar";
import { PanelContent } from "./PanelContent";
import { useStores } from "../../hooks/useStores";
import { Button } from "react-bootstrap";
import { Icon } from "../Icon";
import { Panel as PanelModel } from "../../stores/VisualizationStore";

interface IProps {
  panel: PanelModel;
}

export const Panel: React.FC<IProps> = ({ panel }) => {
  const { visualizationStore: store } = useStores();

  return (
    <div className={styles.panel}>
      <Button
        className={styles.removeButton}
        variant="light"
        onClick={() => store.deletePanel(panel.id)}
      >
        <Icon id="x" />
      </Button>
      <PanelSidebar panel={panel} />
      <PanelContent panel={panel} />
    </div>
  );
};
