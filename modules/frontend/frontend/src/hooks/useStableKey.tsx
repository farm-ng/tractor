import { useState, useEffect } from "react";
import { simpleUniqueId } from "../utils/uniqueId";

// React's reconciler requires a stable, unique key for each element in the list.
// (https://reactjs.org/docs/lists-and-keys.html)
// This hook is useful when rendering a collection of data models without immutable ids.
// You can't use the element's index as its key if you are adding/removing elements from the list.
// And you can't use the element's id as its key, since it is editable by the user.
// So we regenerate unique IDs for each element of the list everytime the length of the list changes.
export function useStableKey<T>(
  values: T[] | undefined
): [string, T][] | undefined {
  const [keyedValues, setKeyedValues] = useState<[string, T][]>();

  useEffect(() => {
    if (values?.length != keyedValues?.length) {
      setKeyedValues(
        values?.map<[string, T]>((value) => [simpleUniqueId(10), value]) || []
      );
    }
  }, [values]);

  return keyedValues;
}
