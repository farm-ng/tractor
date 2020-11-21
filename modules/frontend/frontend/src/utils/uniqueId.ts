// Not enough entropy for large collections or security purposes, but good enough for small ones
export const simpleUniqueId = (length?: number): string =>
  Math.random()
    .toString(36)
    .slice(-(length || 24));
