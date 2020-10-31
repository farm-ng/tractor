// Port of the native Python range() function.
// Based on the https://github.com/jashkenas/underscore implementation.
export function range(start: number, stop?: number, step?: number): number[] {
  if (stop === undefined) {
    stop = start || 0;
    start = 0;
  }
  if (step === 0 || step === undefined) {
    step = stop < start ? -1 : 1;
  }

  const length = Math.max(Math.ceil((stop - start) / step), 0);
  const range = Array(length);

  for (let i = 0; i < length; i++, start += step) {
    range[i] = start;
  }

  return range;
}
