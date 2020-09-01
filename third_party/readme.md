Ensure you have run the bootstrap.sh script which pulls in the system
dependencies.

To build ceres:

```bash
. ../setup.bash
mkdir build-ceres
cd build-ceres
cmake -DCMAKE_INSTALL_PREFIX=$FARM_NG_ROOT/env -DCMAKE_PREFIX_PATH=$FARM_NG_ROOT/env/ -DBUILD_TESTING=OFF -DBUILD_EXAMPLES=OFF ../ceres-solver
make -j$(nproc --ignore=1)
make install
```

To build sophus
```bash
. ../setup.bash
mkdir build-sophus
cd build-sophus
cmake -DCMAKE_INSTALL_PREFIX=$FARM_NG_ROOT/env -DCMAKE_PREFIX_PATH=$FARM_NG_ROOT/env/ -DBUILD_TESTS=OFF -DBUILD_EXAMPLES=OFF ../Sophus
make -j$(nproc --ignore=1)
make install
```

To build apriltag
```bash
. ../setup.bash
mkdir build-apriltag
cd build-apriltag
cmake -DCMAKE_INSTALL_PREFIX=$FARM_NG_ROOT/env -DCMAKE_PREFIX_PATH=$FARM_NG_ROOT/env/ -DCMAKE_BUILD_TYPE=Release ../apriltag
make -j$(nproc --ignore=1)
make install
```
