COMPOSE = docker compose -f docker/docker-compose.yaml

build:
	bash docker/build.bash

start:
	$(COMPOSE) up -d

attach:
	docker exec -it mulsbc-arm64-dev bash

stop:
	$(COMPOSE) down
