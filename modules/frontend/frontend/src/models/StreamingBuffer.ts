/* eslint-disable @typescript-eslint/no-non-null-assertion */
import { decodeAnyEvent } from "./decodeAnyEvent";
import { Event as BusEvent } from "@farm-ng/genproto-core/farm_ng/core/io";
import { Buffer } from "../types/common";
import { EventTypeId } from "../registry/events";
import { TarResourceArchive } from "./ResourceArchive";

export class StreamingBuffer {
  public bufferStart: Date | null = null;
  public bufferEnd: Date | null = null;
  public data: Buffer = {};

  public async loadFromLog(
    resourceArchive: TarResourceArchive,
    logFilePath: string
  ): Promise<void> {
    const blob = await resourceArchive.getBlob(logFilePath);
    const fileBuffer = await blob.arrayBuffer();
    let offset = 0;
    while (offset < fileBuffer.byteLength) {
      const header = fileBuffer.slice(offset, offset + 2);
      const length = new Uint16Array(header)[0];
      offset += 2;
      const record = fileBuffer.slice(offset, offset + length);
      offset += length;
      this.add(BusEvent.decode(new Uint8Array(record)));
    }
  }

  public add(event: BusEvent): void {
    if (!event || !event.data || !event.stamp) {
      return;
    }
    // TODO: Don't assume data is arriving in timestamp order
    this.bufferStart = this.bufferStart || event.stamp;
    this.bufferEnd = event.stamp;
    const typeUrl = event.data.typeUrl as EventTypeId;
    if (!this.data[typeUrl]) {
      this.data[typeUrl] = {};
    }
    if (!this.data[typeUrl]![event.name]) {
      this.data[typeUrl]![event.name] = [];
    }
    const decodedEvent = decodeAnyEvent(event);
    if (decodedEvent) {
      this.data[typeUrl]![event.name].push([
        event.stamp.getTime(),
        decodedEvent,
      ]);
    }
  }

  public clear(): void {
    this.data = {};
    this.bufferStart = null;
    this.bufferEnd = null;
  }
}
