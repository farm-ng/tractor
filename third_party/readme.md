To build librealsense:

```
. ../setup.bash
mkdir build-librealsense
cd build-librealsense
cmake -DBUILD_PYTHON_BINDINGS=true -DCMAKE_BUILD_TYPE=Release -DBUILD_EXAMPLES=false -DCMAKE_INSTALL_PREFIX=$FARM_NG_ROOT/env -DCMAKE_PREFIX_PATH=$FARM_NG_ROOT/env/ ../librealsense/
```

Install udev rules:
```
sudo cp $FARM_NG_ROOT/third_party/librealsense/config/99-realsense-libusb.rules /etc/udev/rules.d/ 
sudo udevadm control --reload-rules && udevadm trigger
```
