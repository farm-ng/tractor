import * as React from "react";
import styles from "./ProgramOutput.module.scss";

interface IProps {
  log: string[];
}

export const ProgramOutput: React.FC<IProps> = ({ log }) => {
  return (
    <div className={styles.programOutput}>
      <pre>
        <code>{String.raw`${log.reverse().join("")}`}</code>
      </pre>
    </div>
  );
};
