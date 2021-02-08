.EXPORT_ALL_VARIABLES:

FARM_NG_DEVEL_IMAGE = farmng/devel@$(shell cat devel/devel-digest.txt)

.base.build.stamp: base.Dockerfile
	docker build --memory=8024m \
	--build-arg FARM_NG_DEVEL_IMAGE=$(FARM_NG_DEVEL_IMAGE) \
	-f base.Dockerfile \
	-t farmng/base:latest \
	-t farmng/base:`git rev-parse --short HEAD` ../ && \
	touch .base.build.stamp

build: .base.build.stamp

.PHONY: build
