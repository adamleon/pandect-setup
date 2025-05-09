#!/bin/bash
set -e

echo "⏳ Installing 3rd party libraries for Event Camera ..."

# Set workspace location
WS_PATH=/workspaces/3d_party_ws
sudo mkdir -p "$WS_PATH/src"
sudo chown -R root:sudo "$WS_PATH"
sudo chmod -R 775 "$WS_PATH"
cd "$WS_PATH/src"

echo "📦 Installing APT packages..."
# Install available packages via apt
sudo apt update
sudo apt install -y \
    ros-${ROS_DISTRO}-metavision-driver \
    ros-${ROS_DISTRO}-event-camera-py \
    python3-vcstool \
    python3-rosdep

echo "📦 Installing packages from source..."
# Import renderer and its dependencies from source using .repos file
git clone =https://github.com/ros-event-camera/event_camera_renderer.git
[ -f event_camera_renderer/event_camera_renderer.repos ] && vcs import < event_camera_renderer/event_camera_renderer.repos ;

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
  echo "source $WS_SETUP_PATH" >> /etc/skel/.bashrc
  echo "🔧 Adding sourcing to /etc/skel/.bashrc for future users"
fi

# Get absolute path to the folder where this script resides
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Install udev rules
UDEV_SRC_DIR="${SCRIPT_DIR}/udev_rules"
UDEV_DST_DIR="/etc/udev/rules.d"

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

echo "✅ Done setting up event camera libraries."


