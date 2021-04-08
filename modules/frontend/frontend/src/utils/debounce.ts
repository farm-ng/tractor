type Timeout = ReturnType<typeof setTimeout>;

export const debounce = <T>(
  callback: (...args: T[]) => void,
  time: number,
  immediate: boolean = false
) => {
  let timeout: Timeout | null;

  return (...args: T[]) => {
    const later = function () {
      timeout = null;
      if (!immediate) callback(...args);
    };
    const callNow = immediate && !timeout;
    if (timeout) clearTimeout(timeout);
    timeout = setTimeout(later, time);
    if (callNow) callback(...args);
  };
};
