const millisecond = 1;
const second = 1000;
const minute = 60 * second;
const hour = 60 * minute;
const day = 24 * hour;

export const duration = {
  millisecond,
  second,
  minute,
  hour,
  day
} as const;
