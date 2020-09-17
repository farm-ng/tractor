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
	make protos
	mkdir -p build
	cd build && cmake .. && make -j$(nproc --ignore=1)

frontend:
	cd app/frontend && yarn && yarn build
	cp -rT app/frontend/dist build/frontend

protos:
	scripts/build-protos.sh

systemd:
	cd jetson && sudo ./install.sh

third_party:
	cd third_party && ./install.sh

test:
	./env.sh pytest $(PY_TEST_FILTER)
	cd app/frontend && yarn test$(JS_TEST_FILTER)

test-integration:
	scripts/test-integration.sh

webserver:
	cd go/webrtc && ../../env.sh make

webservices:
	make protos
	make frontend
	make webserver

.PHONY: bootstrap cpp frontend protos systemd third_party test test-integration webserver webservices all
