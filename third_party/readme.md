To build librealsense:

```bash
. ../setup.bash
mkdir build-librealsense
cd build-librealsense
cmake -DBUILD_PYTHON_BINDINGS=true -DCMAKE_BUILD_TYPE=Release -DBUILD_EXAMPLES=false -DCMAKE_INSTALL_PREFIX=$FARM_NG_ROOT/env -DCMAKE_PREFIX_PATH=$FARM_NG_ROOT/env/ ../librealsense/
```

Install udev rules:
```bash
sudo cp $FARM_NG_ROOT/third_party/librealsense/config/99-realsense-libusb.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && udevadm trigger
```


Ceres deps:
```
sudo apt-get install libgoogle-glog-dev libatlas-base-dev libeigen3-dev libsuitesparse-dev
```

To build ceres:

```bash
. ../setup.bash
mkdir build-ceres
cd build-ceres
cmake -DCMAKE_INSTALL_PREFIX=$FARM_NG_ROOT/env -DCMAKE_PREFIX_PATH=$FARM_NG_ROOT/env/ -DBUILD_TESTING=OFF -DBUILD_EXAMPLES=OFF ../ceres-solver
make -j7
make install
```

To build sophus
```bash
. ../setup.bash
mkdir build-sophus
cd build-sophus
cmake -DCMAKE_INSTALL_PREFIX=$FARM_NG_ROOT/env -DCMAKE_PREFIX_PATH=$FARM_NG_ROOT/env/ -DBUILD_TESTS=OFF -DBUILD_EXAMPLES=OFF ../Sophus
make -j7
make install
```
