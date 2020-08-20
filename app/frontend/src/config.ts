// TODO: Use environment variable to select mock
// import { MockWebSocketClient } from "./clients/MockWebSocketClient";
// const webSocketClient = new MockWebSocketClient();

import { IpcClient } from "./clients/ipc_client";

const ipcClient = new IpcClient("ws://localhost:8989");

const host = "http://localhost";
const port = 8989;

export { ipcClient, host, port };
