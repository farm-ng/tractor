ifdef TEST_FILTER
	PY_TEST_FILTER=-m $(TEST_FILTER)
	JS_TEST_FILTER=:$(TEST_FILTER)
endif

all:
	make bootstrap
	make third_party
	make cpp
	make webservices
	make systemd
	echo "\nSuccessful full system build.\n"

bootstrap:
	./bootstrap.sh

cpp:
	mkdir -p build
	cd build && rm -rf ./* && cmake -DCMAKE_PREFIX_PATH=`pwd`/../env -DCMAKE_BUILD_TYPE=Release .. && make -j`nproc --ignore=1`

frontend:
	cd modules/frontend/frontend && yarn && yarn build
	cp -rT modules/frontend/frontend/dist build/frontend

protos:
	mkdir -p build
	cd build && rm -rf ./* && cmake .. && make -j`nproc --ignore=1` farm_ng_all_protobuf_go farm_ng_all_protobuf_ts

systemd:
	cd jetson && sudo ./install.sh

third_party:
	cd third_party && ./install.sh

test:
	./env.sh pytest $(PY_TEST_FILTER)
	cd app/frontend && yarn test$(JS_TEST_FILTER)

webserver:
	cd modules/frontend/go/webrtc && ../../../../env.sh make

webservices:
	make protos
	make frontend
	make webserver

.PHONY: bootstrap cpp frontend protos systemd third_party test webserver webservices all
