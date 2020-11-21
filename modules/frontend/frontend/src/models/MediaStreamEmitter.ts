type ICallback = (stream: MediaStream) => void;

type mediaStreamEventTypes = "addVideoStream";

type ICallbackMap = {
  [key in mediaStreamEventTypes]: ICallback[];
};

export class MediaStreamEmitter {
  private callbacks: ICallbackMap = { addVideoStream: [] };

  public on(eventType: mediaStreamEventTypes, callback: ICallback): void {
    this.callbacks[eventType] = [
      ...(this.callbacks[eventType] || []),
      callback
    ];
  }

  public addVideoStream(stream: MediaStream): void {
    (this.callbacks["addVideoStream"] || []).forEach((cb) => cb(stream));
  }
}
