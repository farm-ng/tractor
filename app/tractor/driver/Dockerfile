FROM balenalib/jetson-xavier-debian:latest

RUN [ cross-build-start ]
RUN apt-get -q update && apt-get install -yq --no-install-recommends \
	busybox \
	can-utils \
	libsocketcan-dev \
	libsocketcan2 \
	python3-pip \
	python3-setuptools \
	&& apt-get clean && rm -rf /var/lib/apt/lists/*
RUN pip3 install tornado
COPY ./ /opt/farm-ng/src/tractor
RUN [ cross-build-end ]
CMD /opt/farm-ng/src/tractor/env.sh /opt/farm-ng/src/tractor/app/tractor/driver/main.sh
