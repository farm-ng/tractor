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

### Launch the web app
Start webpack - watches the directory and puts build artifacts into ``dist/``:
```
yarn dev-watch
```

Start a tornado backend
```
python sample_backend.py
```

Visit [http://localhost:9010](http://localhost:9010)
