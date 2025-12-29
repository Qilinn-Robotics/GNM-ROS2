#!/bin/bash

if [ "$#" -lt 2 ]; then
    echo "Usage: $0 <mode: rec|simu> <bag_file>"
    echo "  rec:  Play at 1.0x speed (for topomap creation)"
    echo "  simu: Play at 1.0x speed with --loop (for simulation)"
    exit 1
fi

MODE=$1
BAG_FILE=$2

case $MODE in
    rec)
        echo "Playing $BAG_FILE in REC mode (5.0x speed)..."
        ros2 bag play -r 1.0 "$BAG_FILE"
        ;;
    simu)
        echo "Playing $BAG_FILE in SIMU mode (1.0x speed, loop)..."
        ros2 bag play -r 1.0 "$BAG_FILE" --loop
        ;;
    *)
        echo "Invalid mode: $MODE. Use 'rec' or 'simu'."
        exit 1
        ;;
esac
