#!/bin/bash
set -e

# Check if ROS_DISTRO is set, if not, exit with an error
if [ -z "$ROS_DISTRO" ]; then
  echo "❌ Error: ROS_DISTRO environment variable is not set."
  exit 1
fi
# Check if the script is run with a specific directory argument
# If not, use the directory where the script is located
# This allows the script to be run from any location
# and still find the udev_rules directory relative to the script
if [ -n "$1" ]; then
  SCRIPT_DIR="$1"
else
  SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]:-$0}")" && pwd)"
fi
UDEV_SRC_DIR="$SCRIPT_DIR/udev_rules"
UDEV_DST_DIR="/etc/udev/rules.d"

echo "⏳ Installing 3rd party libraries for Event Camera ..."

# Set workspace location
WS_PATH=~/workspaces/3rd_party_ws
sudo mkdir -p "$WS_PATH/src"
sudo chown -R $USER:$USER "$WS_PATH"
sudo chmod -R 775 "$WS_PATH"
cd "$WS_PATH/src"

echo "📦 Installing packages from source..."
# Ensure the workspace src directory is a safe directory for git
if ! git config --global --get safe.directory | grep -q "$WS_PATH/src/event_camera_renderer"; then
  echo "🔧 Adding $WS_PATH/src/event_camera_renderer to git safe directories"
  git config --global --add safe.directory "$WS_PATH/src/event_camera_renderer"
fi

# Import renderer and its dependencies from source using .repos file
if [ ! -d "$WS_PATH/src/event_camera_renderer" ]; then
  echo "🔄 Cloning event_camera_renderer repository..."
  git clone https://github.com/ros-event-camera/event_camera_renderer.git
else
  echo "🔄 Found event_camera_renderer repository"
fi

# Import additional packages from .repos file
if [ ! -f "$WS_PATH/src/event_camera_renderer/event_camera_renderer.repos" ]; then
  echo "⚠️ Warning: event_camera_renderer.repos file not found. Skipping additional package imports."
else
  echo "🔄 Importing additional packages from event_camera_renderer.repos..."

  # Ensure the git safe directory is set for the workspace src
  if ! git config --global --get safe.directory | grep -q "$WS_PATH/src"; then
    echo "🔧 Adding $WS_PATH/src to git safe directories"
    git config --global --add safe.directory "$WS_PATH/src"
  fi

  vcs import --input "$WS_PATH/src/event_camera_renderer/event_camera_renderer.repos" "$WS_PATH/src"
fi

# Return to workspace root
cd "$WS_PATH"

echo "🛠️ Building dependencies and packages..."
# Install ROS 2 dependencies for source packages
rosdep update
rosdep install --from-paths src --ignore-src -r -y

# Build workspace
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo

# Path to workspace
WS_SETUP_PATH="${WS_PATH}/install/setup.bash"

# Ensure sourcing for current user
if ! grep -Fxq "source $WS_SETUP_PATH" "$HOME/.bashrc"; then
  echo "source $WS_SETUP_PATH" >> "$HOME/.bashrc"
  echo "🔧 Adding sourcing to current user's .bashrc"
fi

# Ensure sourcing for future users
if [ -f /etc/skel/.bashrc ] && ! grep -Fxq "source $WS_SETUP_PATH" /etc/skel/.bashrc; then
  echo "source $WS_SETUP_PATH" | sudo tee -a /etc/skel/.bashrc > /dev/null
  echo "🔧 Adding sourcing to /etc/skel/.bashrc for future users"
fi

# Install udev rules
if [ -d "$UDEV_SRC_DIR" ]; then
  echo "🔧 Installing udev rules from: $UDEV_SRC_DIR"
  for rule_file in "$UDEV_SRC_DIR"/*.rules; do
    echo "→ Copying $(basename "$rule_file") to $UDEV_DST_DIR"
    sudo cp "$rule_file" "$UDEV_DST_DIR/"
  done

  echo "🔄 Reloading udev rules..."
  sudo udevadm control --reload-rules
  sudo udevadm trigger

else
  echo "⚠️ Warning: udev_rules directory not found at $UDEV_SRC_DIR"
fi

echo "📦 Installing APT packages..."
# Install available packages via apt
sudo apt update
sudo apt install -y \
    ros-${ROS_DISTRO}-metavision-driver \
    ros-${ROS_DISTRO}-event-camera-py \

echo "✅ Done setting up event camera libraries."


