import { observable } from "mobx";
import { MediaStreamEmitter } from "../models/MediaStreamEmitter";

// A store for information about available media streams
export class MediaStreamStore {
  @observable videoStream: MediaStream | null = null;

  constructor(transport: MediaStreamEmitter) {
    transport.on("addVideoStream", (stream: MediaStream) => {
      this.videoStream = stream;
    });
  }
}
