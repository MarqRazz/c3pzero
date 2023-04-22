# Developers Guide

## Quickly update code repositories

To make sure you have the latest repos:

      cd $COLCON_WS/src/c3pzero
      git checkout main
      git pull origin main
      cd $COLCON_WS/src
      rosdep install --from-paths . --ignore-src -y

## Setup pre-commit

pre-commit is a tool to automatically run formatting checks on each commit, which saves you from manually running clang-format (or, crucially, from forgetting to run them!).

Install pre-commit like this:

```
pip3 install pre-commit
```

Run this in the top directory of the repo to set up the git hooks:

```
pre-commit install
```

## Using ccache
> *Note*: This is already setup in the Docker container

ccache is a useful tool to speed up compilation times with GCC or any other sufficiently similar compiler.

To install ccache on Linux:

    sudo apt-get install ccache

For other OS, search the package manager or software store for ccache, or refer to the [ccache website](https://ccache.dev/)

### Setup

To use ccache after installing it there are two methods. you can add it to your PATH or you can configure it for more specific uses.

ccache must be in front of your regular compiler or it won't be called. It is recommended that you add this line to your `.bashrc`:

    export PATH=/usr/lib/ccache:$PATH

To configure ccache for more particular uses, set the CC and CXX environment variables before invoking make, cmake, catkin_make or catkin build.

For more information visit the [ccache website](https://ccache.dev/).
