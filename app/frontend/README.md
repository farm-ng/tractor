# Frontend Development

### Prerequisites

- Install [Yarn](https://classic.yarnpkg.com/en/docs/install)
- Install [node](https://nodejs.org/en/), with version >12.0.0

### Recommendations

- If using Visual Studio Code, `.vscode/` configuration will handle linting and auto-format on save.

### Install dependencies

```
yarn
```

### Run in development

1. Start any API dependencies

```
cd $FARM_NG_ROOT/go/webrtc
PORT=8081 go run cmd/proxy-server/main.go
```

2. Start

```
cd $FARM_NG_ROOT/app/frontend
BASE_URL="http://tractor.local:8081" yarn dev-start-remote --port 8080
```

### Experimental

- Generate protobuf descriptors
  ```
  yarn run pbjs -t json -o descriptors.json $FARM_NG_ROOT/protos/farm_ng_proto/tractor/v1/*.proto
  ```
