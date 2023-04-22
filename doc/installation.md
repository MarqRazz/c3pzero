# Installation

These instructions assume you are utilizing Docker to build the robot's workspace.

# Setup the C3pzero workspace

1. On the host PC create a workspace that we can share with the docker container (*Note:* Currently the docker container expects this exact workspace `COLCON_WS` name)
``` bash
export COLCON_WS=~/c3pzero_ws/
mkdir -p $COLCON_WS/src
```

2. Get the repo and install any dependencies:
``` bash
cd $COLCON_WS/src
git clone https://github.com/MarqRazz/c3pzero.git
vcs import < c3pzero/c3pzero.repos
```

# Build and run the Docker container

1. Move into the `3cpzero` package where the `docker-compose.yaml` is located and build the docker container with:
``` bash
cd $COLCON_WS/src/c3pzero
docker compose build gpu
```
> *NOTE:* If your machine does not have an Nvidia GPU, run `docker compose build cpu` to build a container without GPU support.

2. Run the Docker container:
``` bash
docker compose run gpu # or cpu
```

3. In a second terminal attach to the container with:
``` bash
docker exec -it c3pzero bash
```

4. Build the colcon workspace with:
``` bash
colcon build --symlink-install --event-handlers log-
```