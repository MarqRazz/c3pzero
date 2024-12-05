# Setup

## Setup the C3pzero workspace

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

# Buildin the Workspace

4. Build the colcon workspace with:
``` bash
colcon build --symlink-install
```
