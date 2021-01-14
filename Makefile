ifdef TEST_FILTER
	PY_TEST_FILTER=-m $(TEST_FILTER)
	JS_TEST_FILTER=:$(TEST_FILTER)
endif

FARM_NG_PREFIX=/farm_ng/env

all:
	make bootstrap
	make third_party
	make cpp
	make webservices
	make systemd
	echo "\nSuccessful full system build.\n"

clean:
	rm -rf build env third_party/build-*

bootstrap:
	./bootstrap.sh


cpp:
	mkdir -p build
	cd build && cmake -DCMAKE_PREFIX_PATH=$(FARM_NG_PREFIX) -DCMAKE_BUILD_TYPE=Release .. && make -j`nproc --ignore=1`

docs:
	mkdir -p build
	cd build && cmake -DCMAKE_PREFIX_PATH=$(FARM_NG_PREFIX) -DBUILD_DOCS=TRUE .. && make docs

frontend:
	cd modules/frontend/frontend && yarn && yarn build
	cp -rT modules/frontend/frontend/dist build/frontend

protos:
	mkdir -p build
	cd build && cmake -DCMAKE_PREFIX_PATH=$(FARM_NG_PREFIX) -DCMAKE_BUILD_TYPE=Release .. && make -j`nproc --ignore=1` farm_ng_all_protobuf_py farm_ng_all_protobuf_go farm_ng_all_protobuf_ts

systemd:
	cd jetson && sudo ./install.sh

third_party:
	cd third_party && ./install.sh

test:
	./env.sh pytest $(PY_TEST_FILTER)
	cd modules/frontend/frontend && yarn test $(JS_TEST_FILTER)

webserver:
	./env.sh make -C modules/frontend/go/webrtc

webservices:
	make protos
	make frontend
	make webserver

.PHONY: clean bootstrap cpp docs frontend protos systemd third_party test webserver webservices all
