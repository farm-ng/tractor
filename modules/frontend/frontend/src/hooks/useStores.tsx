// src/hooks/use-stores.tsx
import * as React from "react";
import { storesContext } from "../contexts";

// eslint-disable-next-line @typescript-eslint/explicit-function-return-type
export const useStores = () => React.useContext(storesContext);
