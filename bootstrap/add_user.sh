#!/bin/bash
set -e

# This script creates a new admin user (sudo).
# Usage: sudo ./add_user.sh <username>

# Check if the script is run as root
if [ "$EUID" -ne 0 ]; then
  echo "Please run as root"
  exit 1
fi

# Check if username is provided
if [ -z "$1" ]; then
  echo "Usage: $0 <username>"
  exit 1
fi

USERNAME="$1"

# Check if user already exists
if id "$USERNAME" &>/dev/null; then
  echo "User '$USERNAME' already exists."
else
  echo "⏳ Creating user '$USERNAME'..."
  adduser --gecos "" "$USERNAME"
fi

# Add user to sudo group
usermod -aG sudo "$USERNAME"

echo "✅ User '$USERNAME' is now an administrator (sudo)"
