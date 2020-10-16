/* eslint-disable no-console */
import { Event as BusEvent } from "../../genproto/farm_ng_proto/tractor/v1/io";
import { BusClient } from "./BusClient";
import { BusEventEmitter } from "./BusEventEmitter";
import { MediaStreamEmitter } from "./MediaStreamEmitter";

export function getWebRTCEmitters(
  endpoint: string
): [BusEventEmitter, MediaStreamEmitter, BusClient] {
  const busEventEmitter = new BusEventEmitter();
  const mediaStreamEmitter = new MediaStreamEmitter();

  const pc = new RTCPeerConnection({
    iceServers: [{ urls: "stun:stun.l.google.com:19302" }]
  });

  pc.ontrack = (event) => {
    if (event.track.kind != "video") {
      console.log(
        `${event.track.kind} track added, but only video tracks are supported.`
      );
      return;
    }
    mediaStreamEmitter.addVideoStream(event.streams[0]);
  };

  pc.oniceconnectionstatechange = (_: Event) =>
    console.log(`New ICE Connection state: ${pc.iceConnectionState}`);

  pc.onicecandidate = async (event: RTCPeerConnectionIceEvent) => {
    if (event.candidate === null) {
      const response = await fetch(endpoint, {
        method: "POST",
        mode: "cors",
        headers: {
          "Content-Type": "application/json"
        },
        body: JSON.stringify({
          sdp: btoa(JSON.stringify(pc.localDescription))
        })
      });

      const responseJson = await response.json();
      try {
        pc.setRemoteDescription(
          new RTCSessionDescription(JSON.parse(atob(responseJson.sdp)))
        );
      } catch (e) {
        console.error(e);
      }
    }
  };

  // Offer to receive 1 video track and a data channel
  pc.addTransceiver("video", { direction: "recvonly" });

  const dataChannel = pc.createDataChannel("data", {
    ordered: false
  });
  dataChannel.onclose = () => console.log("Data channel closed");
  dataChannel.onopen = () => console.log("Data channel opened");
  dataChannel.onmessage = (msg) => {
    const event = BusEvent.decode(new Uint8Array(msg.data));
    busEventEmitter.emit(event);
  };
  const busClient = new BusClient(dataChannel);

  pc.createOffer()
    .then((d) => pc.setLocalDescription(d))
    .catch((e) => console.error(e));

  return [busEventEmitter, mediaStreamEmitter, busClient];
}
