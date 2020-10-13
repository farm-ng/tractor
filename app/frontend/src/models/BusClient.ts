import { Event as BusEvent } from "../../genproto/farm_ng_proto/tractor/v1/io";
import { EventTypeId } from "../registry/events";

export class BusClient {
  constructor(private dataChannel: RTCDataChannel) {}

  public send(typeUrl: EventTypeId, name: string, value: Uint8Array): void {
    const event = {
      ...BusEvent.fromJSON({
        name,
        stamp: new Date()
      }),
      data: { typeUrl, value }
    };
    this.dataChannel.send(BusEvent.encode(event).finish());
  }
}
