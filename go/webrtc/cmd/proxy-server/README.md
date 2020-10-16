#### dev
```
cd go/webrtc
go run cmd/proxy-server/main.go

# enable signaling
go run cmd/proxy-server/main.go -signal=http://192.168.1.137:8586

# with aws server
go run cmd/proxy-server/main.go -signal=http://52.53.120.68:8586
```
