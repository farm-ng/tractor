import * as React from "react";
import * as ReactDOM from "react-dom";
import { Scene } from "./components/Scene";
import { ipcClient } from "./config";

import { Event } from "../genproto/farm_ng_proto/tractor/v1/io";
import { SteeringCommand } from "../genproto/farm_ng_proto/tractor/v1/steering";
import {
  TrackingCameraMotionFrame,
  TrackingCameraPoseFrame
} from "../genproto/farm_ng_proto/tractor/v1/tracking_camera";
import { NamedSE3Pose } from "../genproto/farm_ng_proto/tractor/v1/geometry";
import { MotorControllerState } from "../genproto/farm_ng_proto/tractor/v1/motor";

import "./styles.css";

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

pbmap["type.googleapis.com/farm_ng_proto.tractor.v1.NamedSE3Pose"] =
  NamedSE3Pose.decode;

pbmap["type.googleapis.com/farm_ng_proto.tractor.v1.MotorControllerState"] =
  MotorControllerState.decode;

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

export class App extends React.Component<{}, State> {
  constructor(props: any) {
    super(props);
    this.state = { data: {} };
  }

  public componentDidMount(): any {
    ipcClient.on("*", (ev: Event) => {
      const data = { ...this.state.data };
      data[ev.name] = DecodeEvent(ev);
      this.setState({ data: data });
    });
  }

  public render(): any {
    return (
      <Scene />
      /* <div style={{ flex: 0.5 }}>
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
        </div> */
    );
  }
}

ReactDOM.render(<Scene />, document.getElementById("root"));
