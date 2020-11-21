# Usage

```bash
# install go (tested with version 1.14, where go modules are enabled by default)

# build protobufs and twirp server stubs for Go
make protos

# start the webrtc proxy
cd go/webrtc
go run cmd/proxy-server/main.go

# OR start just the eventbus
go run cmd/eventbus/main.go
```

# Scratchpad

```
gst-launch-1.0 videotestsrc ! videoconvert !  x264enc bitrate=600 speed-preset=ultrafast tune=zerolatency key-int-max=15 ! video/x-h264,profile=constrained-baseline ! queue max-size-time=100000000 ! h264parse !  rtph264pay pt=96 mtu=1400 config-interval=10 ! udpsink host=239.20.20.20 auto-multicast=true port=5000

t265 resolution: 848x800

x264enc bitrate is in kbit/sec, default value is 2048 (2Mbit/s)
omxh264enc bitrate is in bit/sec, default value is 4000000 (4Mbit/s)

The VP8 codec used for video encoding also requires 100–2,000+ Kbit/s of bandwidth, and the bitrate depends on the quality of the streams:
720p at 30 FPS: 1.0~2.0 Mbps
360p at 30 FPS: 0.5~1.0 Mbps
180p at 30 FPS: 0.1~0.5 Mbps

https://stackoverflow.com/questions/7968566/what-would-cause-udp-packets-to-be-dropped-when-being-sent-to-localhost
(env) ian@liszt:~/dev/tractor/build$ sysctl net.core.rmem_max
net.core.rmem_max = 212992
(env) ian@liszt:~/dev/tractor/build$ sysctl net.core.rmem_default
net.core.rmem_default = 212992
(env) ian@liszt:~/dev/tractor/build$ sysctl net.core.wmem_max
net.core.wmem_max = 212992
(env) ian@liszt:~/dev/tractor/build$ sysctl net.core.wmem_default
net.core.wmem_default = 212992
```
