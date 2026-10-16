#!/bin/bash
# Build libTrajDrive.so for gz-sim (Ignition Fortress, gz-sim6) without colcon,
# then install into the default gz system-plugin dir so worlds can load it.
set -e
cd "$(dirname "$0")"

INC="-I/usr/include/ignition/gazebo6 \
     -I/usr/include/ignition/plugin1 \
     -I/usr/include/ignition/common4 \
     -I/usr/include/ignition/math6 \
     -I/usr/include/ignition/msgs8 \
     -I/usr/include/ignition/transport11 \
     -I/usr/include/ignition/utils1 \
     -I/usr/include/ignition/sdformat12 \
     -I/usr/include/ignition/cmake2"

g++ -O2 -fPIC -shared -std=c++17 $INC traj_drive.cc \
    -o libTrajDrive.so \
    -L/usr/lib/x86_64-linux-gnu -lignition-gazebo6 -lignition-plugin1

mkdir -p "$HOME/.ignition/gazebo/plugins"
cp -f libTrajDrive.so "$HOME/.ignition/gazebo/plugins/"
echo "installed -> $HOME/.ignition/gazebo/plugins/libTrajDrive.so"
