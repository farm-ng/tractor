export const toCamelCase = (s: string): string =>
  s
    .replace(/\s(.)/g, (_) => _.toUpperCase())
    .replace(/\s/g, "")
    .replace(/^(.)/, (_) => _.toLowerCase());

export const toSentenceCase = (s: string): string =>
  s
    .replace(/([A-Z])/g, (match) => ` ${match}`)
    .replace(/^./, (match) => match.toUpperCase())
    .trim();

export const truncate = (s: string, len: number): string => {
  if (s.length <= len || len <= 3) {
    return s;
  }
  return s.substring(0, len - 3) + "...";
};
