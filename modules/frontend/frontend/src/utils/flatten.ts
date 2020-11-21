// Given a deeply nested object, returns an object one-level deep
// flatten({ foo: { bar: 'baz' } }) => {foo.bar: "baz"};
// https://gist.github.com/penguinboy/762197
// eslint-disable-next-line @typescript-eslint/no-explicit-any
export function flatten<T extends Record<string, any>>(
  object: T,
  path: string | null = null,
  separator = "."
): T {
  return Object.keys(object).reduce((acc: T, key: string): T => {
    const value = object[key];

    const newPath = [path, key].filter(Boolean).join(separator);

    const isObject = [
      typeof value === "object",
      value !== null,
      !(value instanceof Date),
      !(value instanceof RegExp),
      !(Array.isArray(value) && value.length === 0)
    ].every(Boolean);

    return isObject
      ? { ...acc, ...flatten(value, newPath, separator) }
      : { ...acc, [newPath]: value };
  }, {} as T);
}
