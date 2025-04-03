#!/bin/bash

# Exit on error
set -e

sudo chown -R $(whoami):$(whoami) ~/workspace

cd /home/$(whoami)/workspace

colcon build --symlink-install

echo "source /home/ubuntu/workspace/install/setup.bash" >> ~/.bashrc

echo "Welcome to the Protobot Devcontainer!"