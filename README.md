# pandect-setup
Setup and configurations for pandect on the Khadas VIM3 and NVidia Jetson AGX Orin

Follow the steps below to set up a clean system.

---

## ✅ Step-by-step Installation

Make sure you run all commands from the `bootstrap/` directory:

### 1. Set the hostname

```bash
sudo ./configure_hostname.sh 
```

### 2. Install ROS 2 (choose distribution)

```
sudo ./install_ros.sh [humble|jazzy]
```
Installs:
    ROS 2 Jazzy (Ubuntu 24.04) or Humble (Ubuntu 22.04)
    rosdep, colcon, locale configuration
    Shell sourcing for current and future users

### 3. Install event camera libraries and udev rules
```
sudo ./install_event_camera_libs.sh
```

This script:
    Installs `metavision_driver` and `event_camera_py` via apt
    Clones and builds `event_camera_renderer` and its dependencies via vcs import
    Installs all `.rules` files in `udev_rules/` into `/etc/udev/rules.d/`
    Reloads udev rules immediately

### 4. Create a new admin user

```
sudo ./add_user.sh adamleon
```

Creates the user `adamleon` and adds it to the sudo group.
### 📦 Result

After completing all steps:
- Hostname is set to pandect
- ROS 2 (Humble or Jazzy) is installed and ready
- Event camera libraries and udev rules are configured
- Users are ready to develop

### 🧰 Notes
- These scripts assume a fresh Ubuntu 22.04 (for Humble) or 24.04 (for Jazzy)
- Run all scripts using sudo
- Scripts are not completely safe to re-run