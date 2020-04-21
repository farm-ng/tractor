FROM balenalib/jetson-xavier-debian:latest

# Install build tools and remove layer cache afterwards
RUN apt-get -q update && apt-get install -yq --no-install-recommends \
    	busybox can-utils libsocketcan2 libsocketcan-dev \
	&& apt-get clean && rm -rf /var/lib/apt/lists/*

ENV UDEV=1
COPY ./ /usr/src/app
CMD /usr/src/app/main.sh


