ifdef TEST_FILTER
	PY_TEST_FILTER=-m $(TEST_FILTER)
	JS_TEST_FILTER=:$(TEST_FILTER)
endif

FARM_NG_PREFIX=/farm_ng/env

all:
	make third_party
	make cpp
	make webservices
	make systemd
	echo "\nSuccessful full system build.\n"

clean:
	rm -rf build env third_party/build-*

cmakelists := CMakeLists.txt doc/CMakeLists.txt $(shell find modules/ -type f -name "CMakeLists.txt")

build/CMakeCache.txt: $(cmakelists)
	mkdir -p build && cd build && cmake -DCMAKE_PREFIX_PATH=$(FARM_NG_PREFIX) -DCMAKE_INSTALL_PREFIX=$(FARM_NG_PREFIX) -DCMAKE_BUILD_TYPE=Release -DBUILD_DOCS=TRUE ..

cmake: build/CMakeCache.txt

cpp: build/CMakeCache.txt
	make -C build -j`nproc --ignore=1`

install: build/CMakeCache.txt
	cd build && cmake -DCMAKE_INSTALL_PREFIX=${FARM_NG_PREFIX} ..
	make cpp
	sudo make -C build install

docs: build/CMakeCache.txt
	make -C build docs

frontend:
	cd modules/frontend/frontend && yarn && yarn build
	cp -rT modules/frontend/frontend/dist build/frontend

protos: build/CMakeCache.txt
	make -C build -j`nproc --ignore=1` farm_ng_all_protobuf_py farm_ng_all_protobuf_go farm_ng_all_protobuf_ts

systemd:
	cd jetson && sudo ./install.sh

test:
	pytest $(PY_TEST_FILTER)
	cd modules/frontend/frontend && yarn test $(JS_TEST_FILTER)

webserver:
	make -C modules/frontend/go/webrtc

webservices:
	make protos
	make frontend
	make webserver

.PHONY: clean bootstrap cpp docs frontend protos systemd third_party test webserver webservices cmake all
