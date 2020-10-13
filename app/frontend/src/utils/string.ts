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
