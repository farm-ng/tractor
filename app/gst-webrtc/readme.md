This directory is copied from the gstreamer project, and is under and LGPL license (see the COPYING file next to this readme.)

Original source:
https://gitlab.freedesktop.org/gstreamer/gst-examples/-/blob/master/webrtc/sendonly/


Build the tractor project and then run tracking camera:

```bash
$ ~/tractor/build/cpp/farm_ng/farm-ng-tracking_camera
```

This will start up the t265 frame grabber and publish an RTP stream on a UDP unicast port.

Then build this webrtc example using make, and run it from from this directory:

```
farmer@tractor01:~$ cd tractor/app/gst-webrtc/
farmer@tractor01:~/tractor/app/gst-webrtc$ make
"gcc" -O0 -ggdb -Wall -fno-omit-frame-pointer webrtc-unidirectional-h264.c -pthread -I/usr/include/gstreamer-1.0 -I/usr/include/libsoup-2.4 -I/usr/include/libxml2 -I/usr/include/json-glib-1.0 -I/usr/include/glib-2.0 -I/usr/lib/aarch64-linux-gnu/glib-2.0/include -lgstwebrtc-1.0 -lgstbase-1.0 -lgstreamer-1.0 -lgstsdp-1.0 -lsoup-2.4 -ljson-glib-1.0 -lgio-2.0 -lgobject-2.0 -lglib-2.0 -o webrtc-unidirectional-h264
farmer@tractor01:~/tractor/app/gst-webrtc$ ./webrtc-unidirectional-h264 
WebRTC page link: http://127.0.0.1:57778/
```

If you don't have the t265, you can test by running this pipeline instead of farm-ng-tracking_camera:
```
gst-launch-1.0 v4l2src ! videoconvert ! omxh264enc control-rate=1 bitrate=2500000 ! video/x-h264, stream-format=byte-stream ! rtph264pay pt=96 mtu=1400 config-interval=10 ! udpsink host=239.20.20.22 auto-multicast=true  port=5000
```


