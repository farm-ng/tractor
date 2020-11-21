import * as React from "react";
import { Header } from "./Header";
import { Content } from "./Content";
import styles from "./Root.module.scss";

export const Root: React.FC = () => {
  return (
    <div className={styles.root}>
      <Header />
      <Content />
    </div>
  );
};
