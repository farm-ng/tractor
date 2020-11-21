import { useRef, useLayoutEffect, useState, useCallback } from "react";
import ResizeObserver from "resize-observer-polyfill";

export const useResizeObserver = (): [
  HTMLDivElement | null,
  React.Dispatch<React.SetStateAction<HTMLDivElement | null>>,
  ResizeObserverEntry | undefined
] => {
  const [observed, setObserved] = useState<HTMLDivElement | null>(null);
  const observer = useRef<ResizeObserver | null>(null);
  const [observation, setObservation] = useState<ResizeObserverEntry>();

  const disconnect = useCallback(() => observer.current?.disconnect(), []);

  const observe = useCallback(() => {
    observer.current = new ResizeObserver(([entry]) => setObservation(entry));
    if (observed) {
      observer.current.observe(observed);
    }
  }, [observed]);

  useLayoutEffect(() => {
    observe();
    return () => disconnect();
  }, [disconnect, observe]);

  return [observed, setObserved, observation];
};
