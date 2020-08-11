// The following hack is a work around for this issue:
// https://github.com/stephenh/ts-proto/issues/108
import * as protobuf from "protobufjs/minimal";
import * as Long from "long";
protobuf.util.Long = Long;
protobuf.configure();

import * as React from "react";
import * as ReactDOM from "react-dom";

import { Event } from "../genproto/farm_ng_proto/tractor/v1/io";

export type State = {
  readonly data: Event;
};

class App extends React.Component<{}, State> {
  constructor(props: any) {
    super(props);
    this.state = { data: Event.fromJSON({}) };
  }

  public componentDidMount() {
    const ws = new WebSocket("ws://localhost:8989");
    ws.binaryType = "arraybuffer";

    ws.onmessage = (ev: MessageEvent) => {
      const pbEvent = Event.decode(new Uint8Array(ev.data));
      this.setState({ data: pbEvent });
    };
  }

  public render() {
    return <div>{JSON.stringify(Event.toJSON(this.state.data))} </div>;
  }
}

ReactDOM.render(<App />, document.getElementById("root"));
