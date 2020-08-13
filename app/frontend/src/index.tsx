// The following hack is a work around for this issue:
// https://github.com/stephenh/ts-proto/issues/108
import * as protobuf from "protobufjs/minimal";
import * as Long from "long";
protobuf.util.Long = Long;
protobuf.configure();

import * as React from "react";
import * as ReactDOM from "react-dom";
import { Scene } from "./components/Scene";
import { Event } from "../genproto/farm_ng_proto/tractor/v1/io";
import { SteeringCommand } from "../genproto/farm_ng_proto/tractor/v1/steering";
import {
  TrackingCameraMotionFrame,
  TrackingCameraPoseFrame
} from "../genproto/farm_ng_proto/tractor/v1/tracking_camera";

type ProtobufAnyMap = {
  [key: string]: (data: Uint8Array) => any;
};

const pbmap: ProtobufAnyMap = {};

pbmap["type.googleapis.com/farm_ng_proto.tractor.v1.SteeringCommand"] =
  SteeringCommand.decode;

pbmap["type.googleapis.com/farm_ng_proto.tractor.v1.TrackingCameraPoseFrame"] =
  TrackingCameraPoseFrame.decode;

pbmap[
  "type.googleapis.com/farm_ng_proto.tractor.v1.TrackingCameraMotionFrame"
] = TrackingCameraMotionFrame.decode;

function DecodeEvent(msg: Event): any {
  if (!msg.data) {
    return {};
  }
  if (msg.data.typeUrl in pbmap) {
    return pbmap[msg.data.typeUrl](msg.data.value);
  } else {
    return {};
  }
}
type EventMap = {
  [key: string]: any;
};

export type State = {
  readonly data: EventMap;
};

class App extends React.Component<{}, State> {
  constructor(props: any) {
    super(props);
    this.state = { data: {} };
  }

  public componentDidMount() {
    const ws = new WebSocket("ws://localhost:8989");
    ws.binaryType = "arraybuffer";
    ws.onmessage = (ev: MessageEvent) => {
      const pbEvent = Event.decode(new Uint8Array(ev.data));
      const data = { ...this.state.data };
      data[pbEvent.name] = DecodeEvent(pbEvent);
      this.setState({ data: data });
    };
  }

  public render() {
    return (
      <div>
        <Scene />

        <div>
          {Object.keys(this.state.data).map((key, i) => (
            <p key={i}>
              <span>Key Name: {key} </span>
              <p>
                {Object.keys(this.state.data[key]).map((key_j, _j) => (
                  <p>
                    <span> {key_j} </span>
                    <span> {JSON.stringify(this.state.data[key][key_j])} </span>
                  </p>
                ))}
              </p>
            </p>
          ))}
        </div>
      </div>
    );
  }
}

ReactDOM.render(<App />, document.getElementById("root"));
