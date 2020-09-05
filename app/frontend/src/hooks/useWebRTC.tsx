/* eslint-disable no-console */
import { useEffect } from "react";
import { Event as BusEvent } from "../../genproto/farm_ng_proto/tractor/v1/io";
import { BusEventEmitter } from "../models/BusEventEmitter";

export function useWebRTC(
  endpoint: string,
  videoRef: React.MutableRefObject<HTMLVideoElement | null>
): BusEventEmitter {
  const emitter = new BusEventEmitter();
  useEffect(() => {
    const videoElement = videoRef?.current;
    if (!videoElement) {
      return;
    }

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
      videoElement.srcObject = event.streams[0];
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

    const dataChannel = pc.createDataChannel("foo"); // TODO: investigate data channel naming
    dataChannel.onclose = () => console.log("Data channel closed");
    dataChannel.onopen = () => console.log("Data channel opened");
    dataChannel.onmessage = (msg) => {
      const event = BusEvent.decode(new Uint8Array(msg.data));
      emitter.emit(event);
    };

    pc.createOffer()
      .then((d) => pc.setLocalDescription(d))
      .catch((e) => console.error(e));
  }, [videoRef]);

  return emitter;
}
