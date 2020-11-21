import { useState, useEffect } from "react";

interface IProps<T> {
  initialValue: T;
  onChange: (updated: T) => void;
}

export function useFormState<T>({
  initialValue, // initial value for `value`
  onChange // automatically invoked whenever `value` changes
}: IProps<T>): [T, (updater: (c: T) => T) => void] {
  const [value, setValue] = useState(initialValue);

  useEffect(() => {
    if (value) {
      onChange(value);
    }
  }, [value]);

  return [value, setValue];
}
