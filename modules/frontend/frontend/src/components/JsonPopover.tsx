import * as React from "react";
import ReactJson from "react-json-view";
import { Button, OverlayTrigger, Popover } from "react-bootstrap";
import styles from "./JsonPopover.module.scss";
import { OverlayTriggerType } from "react-bootstrap/esm/OverlayTrigger";

interface IProps {
  // eslint-disable-next-line @typescript-eslint/ban-types
  json: Object;
  title?: string;
  trigger?: "click" | "hover";
  collapsed?: number;
}

export const JsonPopover: React.FC<IProps> = ({
  title,
  json,
  children,
  collapsed,
  ...props
}) => {
  const jsonElement = (
    <ReactJson src={json} displayDataTypes={false} collapsed={collapsed || 1} />
  );

  const trigger: OverlayTriggerType[] =
    props.trigger === "hover" ? ["hover", "focus"] : ["click"];

  const popover = (
    <Popover>
      {title && <Popover.Title as="h3">{title}</Popover.Title>}
      <Popover.Content>
        <div className={styles.popoverContent}>{jsonElement}</div>
      </Popover.Content>
    </Popover>
  );

  return (
    <OverlayTrigger trigger={trigger} placement="auto" overlay={popover}>
      {children || <Button variant={"light"}>{"{}"}</Button>}
    </OverlayTrigger>
  );
};
