#!/bin/bash
set -e
# Change the hostname of the system to 'pandect'
# This script sets the hostname to 'pandect' and updates /etc/hosts accordingly.
# It is assumed that the script is run with root privileges.

# Check if the script is run as root
if [ "$EUID" -ne 0 ]; then
    echo "Please run as root"
    exit 1
fi

NEW_HOSTNAME="pandect"
CURRENT_HOSTNAME="$(hostname)"

echo "⏳ Changing hostname from '$CURRENT_HOSTNAME' to '$NEW_HOSTNAME'..."

# Write new hostname to /etc/hostname
echo "$NEW_HOSTNAME" > /etc/hostname

# Apply hostname immediately
hostnamectl set-hostname "$NEW_HOSTNAME"

# Update /etc/hosts with new hostname
sed -i "s/$CURRENT_HOSTNAME/$NEW_HOSTNAME/g" /etc/hosts

echo "✅ Hostname successfully set to '$NEW_HOSTNAME'"
