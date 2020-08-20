// The following hack is a work around for this issue:
// https://github.com/stephenh/ts-proto/issues/108

import * as protobuf from "protobufjs/minimal";
import * as Long from "long";

if (protobuf.util.Long !== Long) {
  protobuf.util.Long = Long;
  protobuf.configure();
}

import { Event } from "../../genproto/farm_ng_proto/tractor/v1/io";

type ICallback = (event: Event) => void;

type ICallbackMap = {
  [event: string]: ICallback[];
};

export class IpcClient {
  private websocket: WebSocket;
  private callbacks: ICallbackMap = {};

  constructor(uri: string) {
    this.websocket = new WebSocket(uri);
    this.websocket.binaryType = "arraybuffer";

    this.websocket.onmessage = (ev: MessageEvent): void => {
      const pbEvent = Event.decode(new Uint8Array(ev.data));
      this.emit(pbEvent);
    };
  }

  public on(event: string, callback: ICallback): void {
    this.callbacks[event] = [...(this.callbacks[event] || []), callback];
  }

  private emit(event: Event): void {
    (this.callbacks["*"] || []).forEach((cb) => cb(event));
    if (event.data) {
      (this.callbacks[event.data.typeUrl] || []).forEach((cb) => cb(event));
    }
  }
}
