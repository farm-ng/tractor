# Frontend Development

### Prerequisites

- [Yarn](https://classic.yarnpkg.com/en/docs/install)
- [node](https://nodejs.org/en/), version 12+

### Recommendations

- If using Visual Studio Code, `.vscode/` configuration will handle linting and auto-format on save.

### Install dependencies

```
yarn
```

### Run in development

In production, the API and SPA are served by the same server.

In development it's helpful to run them separately.

1. Start the API

```
cd $FARM_NG_ROOT/modules/frontend/go/webrtc
PORT=8081 go run cmd/proxy-server/main.go
```

2. Start the SPA (with `webpack-dev-server` in watch mode)

```
cd $FARM_NG_ROOT/modules/frontend/frontend
BASE_URL="http://tractor.local:8081" yarn dev-start-remote --port 8080
```


### Concepts

- `Visualizers` operate on a `TimestampedEventVector`: `[[t0, e0], [t1, e1], [t2, e2], ...]`
- Depending on the context, timestamps may be interpreted as wall-time, an index number, or undefined (if no time metric is relevant).
- The events in a `TimestampedEventVector` are of the same type. This type may be any of the protobuf messages we support, including the generic `Event` message type. The `Event` message encapsulates any of the other messages types, via an `Any` field. So via `Events` we get polymorphism.
- Some `Visualizers` may integrate information across the `TimestampedEventVector` (e.g. to plot multiple values, or calculate a moving average). Others simply provide a mechanism for scanning through visualization of single elements.
- The `Oscilloscope` view allows the user to filter eventbus traffic by event _type_, then choose a `Visualizer` registered for that event type.
- Multiple visualizers may be registered for a given event type. The visualizer registered with the highest priority is that event type's `preferred visualizer`. Some visualizers are registered for all event types (e.g. the `JSONVisualizer`) -- the one of these with the highest priority is the `default visualizer`.
- Visualizers may expose options.
- The `Programs` view allows programs to define a filter on eventbus traffic by event _name_. It then uses the generic `Event` visualizer, which displays the preferred visualization for each event in the vector.
- Note, `Visualizers` are distinct from the library of UI components we use to help us build visualizers. For example, the `Plot` is a UI component, while the `SteeringCommandVisualizer` is a visualizer that uses `Plots` to display velocities.


### Experimental

- Generate protobuf descriptors
  ```
  yarn run pbjs -t json -o descriptors.json $FARM_NG_ROOT/modules/<module>/protos/farm_ng/<module>/*.proto
  ```
