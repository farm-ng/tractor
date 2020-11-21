import * as React from "react";
import { Scene } from "./Scene";
import styles from "./Map.module.scss";

export const Map: React.FC = () => {
  return (
    <div className={styles.map}>
      <Scene />
    </div>
  );
};
