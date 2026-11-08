#!/usr/bin/env bash
set -euo pipefail
# Run in your own terminal so sudo can request your password locally.
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DOWNLOAD_CACHE="$(cd "$ROOT/../../work/apt-cache" && pwd)/archives"
# Copy only completed archives; background downloads use their own cache.
sudo mkdir -p /var/cache/apt/archives
for deb in "$DOWNLOAD_CACHE"/*.deb; do
  if [[ -f "$deb" && ! -e "/var/cache/apt/archives/$(basename "$deb")" ]]; then
    sudo cp "$deb" /var/cache/apt/archives/
  fi
done
sudo apt-get -o Acquire::ForceIPv4=true update
sudo apt-get -o Acquire::ForceIPv4=true install -y ros-jazzy-moveit ros-jazzy-ros2-control \
  ros-jazzy-ros2-controllers ros-jazzy-gz-ros2-control \
  ros-jazzy-camera-calibration python3-venv python3-pip python3-serial python3-opencv
sudo usermod -aG dialout "$(id -un)"
echo 'System packages installed. Log out and back in for USB serial permissions.'
echo "Then: source $ROOT/activate.sh"
