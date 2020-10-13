import * as React from "react";
import { useState } from "react";
import { Card as BootstrapCard, Collapse } from "react-bootstrap";
import { formatValue } from "../../../utils/formatValue";
import { JsonPopover } from "../../JsonPopover";
import styles from "./Card.module.scss";

interface IProps {
  title?: string;
  timestamp?: number;
  // eslint-disable-next-line @typescript-eslint/ban-types
  json?: Object;
  collapsed?: boolean;
  className?: string;
}

export const Card: React.FC<IProps> = ({
  title,
  timestamp,
  json,
  children,
  className,
  ...props
}) => {
  const showFooter = json || timestamp;
  const [collapsed, setCollapsed] = useState<boolean>(Boolean(props.collapsed));
  const toggleCollapsed = (): void => setCollapsed((prev) => !prev);
  return (
    <BootstrapCard bg={"light"} className={["shadow-sm", className].join(" ")}>
      <BootstrapCard.Header className={styles.header} onClick={toggleCollapsed}>
        {title}
      </BootstrapCard.Header>
      <Collapse in={!collapsed}>
        <BootstrapCard.Body>{children}</BootstrapCard.Body>
      </Collapse>
      {showFooter && (
        <BootstrapCard.Footer className={styles.footer}>
          {timestamp !== undefined && (
            <span className="text-muted">
              {formatValue(new Date(timestamp))}
            </span>
          )}
          {json && <JsonPopover json={json} />}
        </BootstrapCard.Footer>
      )}
    </BootstrapCard>
  );
};
