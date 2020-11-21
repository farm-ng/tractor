import * as React from "react";
import {
  Button,
  Form as BootstrapForm,
  FormControlProps,
  FormProps
} from "react-bootstrap";
import { ButtonProps } from "react-bootstrap/esm/Button";
import { toCamelCase } from "../../../utils/string";
import { simpleUniqueId } from "../../../utils/uniqueId";
import styles from "./Form.module.scss";

interface IGroupProps extends FormControlProps {
  label: string;
  id?: string;
  description?: string;
  checked?: boolean;
  onChange: React.ChangeEventHandler<
    HTMLInputElement | HTMLSelectElement | HTMLTextAreaElement
  >;
}

const Group: React.FC<IGroupProps> = ({
  label,
  value,
  id,
  description,
  children,
  ...props
}) => {
  const controlId = id || `${toCamelCase(label)}_${simpleUniqueId(4)}`;
  const defaultValue = typeof value === "number" ? 0 : "";
  return (
    <BootstrapForm.Group controlId={controlId}>
      <BootstrapForm.Label>{label}</BootstrapForm.Label>
      <BootstrapForm.Control
        name={controlId}
        value={value || defaultValue}
        step={props.type === "number" ? "any" : undefined}
        {...props}
      >
        {children}
      </BootstrapForm.Control>
      {description && (
        <BootstrapForm.Text className="text-muted">
          {description}
        </BootstrapForm.Text>
      )}
    </BootstrapForm.Group>
  );
};

interface IButtonGroupProps extends ButtonProps {
  buttonText: string;
  onClick?: () => void;
}

const ButtonGroup: React.FC<IButtonGroupProps> = ({ buttonText, ...props }) => {
  return (
    <BootstrapForm.Group>
      <BootstrapForm.Label>&nbsp;</BootstrapForm.Label>
      <div>
        <Button {...props}>{buttonText}</Button>
      </div>
    </BootstrapForm.Group>
  );
};

interface IFormProps extends FormProps {
  onSubmit: (e: React.FormEvent<HTMLFormElement>) => void;
}

const Form: React.FC<IFormProps> = (props) => {
  return (
    <BootstrapForm {...props} className={styles.form}>
      {props.children}
    </BootstrapForm>
  );
};

export default Object.assign(Form, { Group, ButtonGroup });
