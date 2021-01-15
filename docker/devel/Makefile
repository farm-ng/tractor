.EXPORT_ALL_VARIABLES:

FARM_NG_DEVEL_IMAGE = farmng/devel@$(shell cat devel-digest.txt)
uid = $(shell id -u)

test:
	env
down:
	@docker-compose down

.user_devel.build.stamp: docker-compose.yml user_devel.Dockerfile
	@docker-compose down
	@docker-compose build && touch .user_devel.build.stamp

build: .user_devel.build.stamp

up: .user_devel.build.stamp
	@docker-compose up --remove-orphans

upd: .user_devel.build.stamp
	@docker-compose up -d --remove-orphans && \
	echo "farm-ng's devel container is running.  Run ``make down`` to stop it." && \
	echo "You can open up a shell inside the container:" && \
	echo "   docker exec -it devel_workspace_1 bash"

docs: .user_devel.build.stamp
	@docker-compose run --rm workspace make docs

.PHONY: upd down up build docs
