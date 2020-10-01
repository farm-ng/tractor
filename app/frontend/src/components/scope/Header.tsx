/* eslint-disable no-console */
import * as React from "react";
import styles from "./Header.module.scss";
import { Button, Col, Container, Form } from "react-bootstrap";
import { useObserver } from "mobx-react-lite";
import RangeSlider from "react-bootstrap-range-slider";
import { useStores } from "../../hooks/useStores";
import { TarResourceArchive } from "../../models/ResourceArchive";
import { StreamingBuffer } from "../../models/StreamingBuffer";
import { formatValue } from "../../utils/formatValue";
import { Icon } from "../Icon";
import { duration } from "../../utils/duration";
import { normalizeTarPath } from "../../models/TarReader";

export const Header: React.FC = () => {
  const { visualizationStore: store } = useStores();
  const fileInputRef = React.useRef<HTMLInputElement | null>(null);
  const [logFilePath, setLogFilePath] = React.useState<string | null>(null);

  const handleOnLoadLog = async (
    e: React.ChangeEvent<HTMLInputElement>
  ): Promise<void> => {
    store.bufferLogLoadProgress = 0;
    const eventTarget = e.target;
    const file = eventTarget.files?.[0];
    if (file) {
      const resourceArchive = new TarResourceArchive(file);
      const fileInfo = await resourceArchive.getFileInfo();
      // As a heuristic, find the shortest filename ending in .log
      const logFilePath = fileInfo
        .filter((_) => _.endsWith(".log"))
        .sort((a, b) => a.length - b.length)[0];
      console.log("Log file path: ", logFilePath);
      console.log("Tarball contents: ", fileInfo);
      if (!logFilePath) {
        throw Error("No .log file in archive");
      }
      const normalizedLogFilePath = normalizeTarPath(logFilePath);
      setLogFilePath(normalizedLogFilePath);
      const streamingBuffer = new StreamingBuffer();
      await streamingBuffer.loadFromLog(resourceArchive, normalizedLogFilePath);
      store.replaceBuffer(streamingBuffer);
      store.resourceArchive = resourceArchive;
      store.bufferLogLoadProgress = 1;
      eventTarget.value = "";
    }
  };

  return useObserver(() => {
    return (
      <div className={styles.header}>
        <Container fluid>
          <Form.Row>
            <Col xs={2}>
              <div className={styles.dataSourceButtons}>
                <Button
                  onClick={() => store.toggleStreaming()}
                  className={[
                    styles.streamButton,
                    store.isStreaming && styles.active
                  ].join(" ")}
                >
                  <Icon id={store.isStreaming ? "stop" : "play"} />
                  {store.isStreaming ? "Stop" : "Stream"}
                </Button>

                <Button
                  disabled={store.isStreaming}
                  onClick={() => fileInputRef.current?.click()}
                >
                  Load log
                </Button>
                <input
                  ref={fileInputRef}
                  onChange={handleOnLoadLog}
                  name="loadLog"
                  type="file"
                  hidden
                />
              </div>
              <div className={"text-muted"}>
                {store.bufferLogLoadProgress === 1 && `${logFilePath}`}
              </div>
            </Col>
            <Col xs={8}>
              <RangeSlider
                step={0.01}
                max={1}
                value={store.bufferRangeStart}
                disabled={store.bufferEmpty}
                tooltip={
                  !store.isStreaming && store.bufferStart && store.bufferEnd
                    ? "auto"
                    : "off"
                }
                tooltipLabel={() => formatValue(store.bufferRangeStartDate)}
                onChange={(e) =>
                  store.setBufferRangeStart(parseFloat(e.target.value))
                }
              />
              <RangeSlider
                step={0.01}
                max={1}
                value={store.bufferRangeEnd}
                disabled={store.bufferEmpty}
                tooltip={
                  !store.isStreaming && store.bufferStart && store.bufferEnd
                    ? "auto"
                    : "off"
                }
                tooltipLabel={() => formatValue(store.bufferRangeEndDate)}
                onChange={(e) =>
                  store.setBufferRangeEnd(parseFloat(e.target.value))
                }
              />
              <div className={styles.bufferTimestamps}>
                {store.bufferStart && (
                  <span className={"text-muted"}>
                    {formatValue(store.bufferStart)}
                  </span>
                )}
                {store.bufferEnd && (
                  <span className={"text-muted"}>
                    {formatValue(store.bufferEnd)}
                  </span>
                )}
              </div>
            </Col>
            <Col xs={1}>
              <Form.Group controlId="buffer.throttle">
                <Form.Label>Throttle</Form.Label>
                <Form.Control
                  as="select"
                  disabled={store.bufferEmpty}
                  value={store.bufferThrottle}
                  onChange={(e: React.ChangeEvent<HTMLSelectElement>) =>
                    (store.bufferThrottle = parseInt(e.target.value))
                  }
                >
                  <option value={0}>none</option>
                  <option value={1}>1 ms</option>
                  <option value={100}>100 ms</option>
                  <option value={500}>500 ms</option>
                  <option value={duration.second}>1 s</option>
                </Form.Control>
              </Form.Group>
            </Col>
            <Col xs={1}>
              <Form.Group controlId="buffer.length">
                <Form.Label>Expiration</Form.Label>
                <Form.Control
                  as="select"
                  disabled={store.bufferEmpty || !store.isStreaming}
                  value={store.bufferExpirationWindow}
                  onChange={(e: React.ChangeEvent<HTMLSelectElement>) =>
                    (store.bufferExpirationWindow = parseInt(e.target.value))
                  }
                >
                  <option value={20 * duration.second}>20 s</option>
                  <option value={duration.minute}>1 min</option>
                  <option value={5 * duration.minute}>5min</option>
                  <option value={30 * duration.minute}>30min</option>
                </Form.Control>
              </Form.Group>
            </Col>
          </Form.Row>
        </Container>
      </div>
    );
  });
};
