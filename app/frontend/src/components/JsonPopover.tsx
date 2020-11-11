import * as React from "react";
import ReactJson from "react-json-view";
import { Button, OverlayTrigger, Popover } from "react-bootstrap";
import styles from "./JsonPopover.module.scss";

interface IProps {
  title?: string;
  // eslint-disable-next-line @typescript-eslint/ban-types
  json: Object;
}

export const JsonPopover: React.FC<IProps> = ({ title, json }) => {
  const jsonElement = (
    <ReactJson src={json} displayDataTypes={false} collapsed={1} />
  );

  const popover = (
    <Popover>
      {title && <Popover.Title as="h3">{title}</Popover.Title>}
      <Popover.Content>
        <div className={styles.popoverContent}>{jsonElement}</div>
      </Popover.Content>
    </Popover>
  );

  return (
    <OverlayTrigger trigger="click" placement="auto" overlay={popover}>
      <Button variant={"light"}>{"{}"}</Button>
    </OverlayTrigger>
  );
};
