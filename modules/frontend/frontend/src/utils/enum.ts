/* eslint-disable @typescript-eslint/no-explicit-any */
export const enumNumericKeys = (_enum: Record<string, any>): number[] =>
  Object.keys(_enum)
    .map((k) => parseInt(k))
    .filter((x) => !isNaN(x));

export const enumStringValues = (_enum: Record<string, any>): string[] =>
  Object.values(_enum).filter((v: any) => typeof v === "string");
