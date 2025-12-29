#!/bin/bash

# Get the directory of this script
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

# Set path to the src directory where topic_names.py is located
SRC_DIR="$DIR/../../src"

# Check if topic_names.py exists
if [ ! -f "$SRC_DIR/topic_names.py" ]; then
    echo "Error: topic_names.py not found in $SRC_DIR"
    exit 1
fi

# Extract the IMAGE_TOPIC variable using python
IMAGE_TOPIC=$(PYTHONPATH="$SRC_DIR" python3 -c "from topic_names import IMAGE_TOPIC; print(IMAGE_TOPIC)")

if [ -z "$IMAGE_TOPIC" ]; then
    echo "Error: Failed to retrieve IMAGE_TOPIC from topic_names.py"
    exit 1
fi

echo "Read IMAGE_TOPIC from topic_names.py: $IMAGE_TOPIC"

# Define the output bag name
if [ -n "$1" ]; then
    BAG_NAME="$1"
else
    BAG_NAME="gnm_bag_$(date +%Y-%m-%d-%H-%M-%S).bag"
fi

echo "Starting rosbag record for topic: $IMAGE_TOPIC"
echo "Output file: $BAG_NAME"

# Execute rosbag record
ros2 bag record -o "$BAG_NAME" "$IMAGE_TOPIC"
