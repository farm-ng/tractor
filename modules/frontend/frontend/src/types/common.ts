import { Writer, Reader } from "protobufjs/minimal";
import { EventType, EventTypeId } from "../registry/events";

type Timestamp = number;

export type TimestampedEvent<T extends EventType = EventType> = [Timestamp, T];

export type TimestampedEventVector<
  T extends EventType = EventType
> = TimestampedEvent<T>[];

export type Buffer = {
  [k in EventTypeId]?: { [k: string]: TimestampedEventVector };
};

type Builtin = Date | Function | Uint8Array | string | number | undefined;

type DeepPartial<T> = T extends Builtin
  ? T
  : T extends Array<infer U>
  ? Array<DeepPartial<U>>
  : T extends ReadonlyArray<infer U>
  ? ReadonlyArray<DeepPartial<U>>
  : T extends {}
  ? { [K in keyof T]?: DeepPartial<T[K]> }
  : Partial<T>;

export type Message<T> = {
  encode(message: T, writer: Writer): Writer;
  decode(input: Uint8Array | Reader, length?: number): T;
  // eslint-disable-next-line @typescript-eslint/no-explicit-any
  fromJSON(object: any): T;
  fromPartial(object: DeepPartial<T>): T;
  toJSON(message: T): unknown;
};
