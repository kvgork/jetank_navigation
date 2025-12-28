#!/bin/bash
# Save the current SLAM map to a file
#
# Usage:
#   ./save_map.sh <map_name>
#   ./save_map.sh              (defaults to 'jetank_map')
#
# This script saves the map in two formats:
#   - .yaml (map metadata)
#   - .pgm  (map image)

# Default map name
MAP_NAME=${1:-jetank_map}

# Default save location
MAP_DIR="${HOME}/maps"

# Create maps directory if it doesn't exist
mkdir -p "${MAP_DIR}"

echo "========================================="
echo "JeTank Map Saver"
echo "========================================="
echo "Map name: ${MAP_NAME}"
echo "Save location: ${MAP_DIR}"
echo ""

# Check if map_server is running
if ! ros2 topic list | grep -q "/map"; then
    echo "ERROR: /map topic not found!"
    echo "Make sure SLAM Toolbox is running."
    exit 1
fi

# Save the map
echo "Saving map..."
ros2 run nav2_map_server map_saver_cli -f "${MAP_DIR}/${MAP_NAME}"

# Check if save was successful
if [ $? -eq 0 ]; then
    echo ""
    echo "========================================="
    echo "Map saved successfully!"
    echo "========================================="
    echo "Files created:"
    echo "  YAML: ${MAP_DIR}/${MAP_NAME}.yaml"
    echo "  PGM:  ${MAP_DIR}/${MAP_NAME}.pgm"
    echo ""
    echo "To use this map with Nav2:"
    echo "  ros2 launch jetank_navigation nav2_bringup.launch.py \\"
    echo "    map:=${MAP_DIR}/${MAP_NAME}.yaml"
    echo "========================================="
else
    echo ""
    echo "ERROR: Map save failed!"
    exit 1
fi
