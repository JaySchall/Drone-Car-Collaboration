#!/bin/bash

# Find the PID of the target Python process using pgrep
pid=$(pgrep -f "python3 car_command_server.py")

# Check if a matching process was found
if [ -n "$pid" ]; then
    echo "Killing Python3 process with PID: $pid"
    kill -9 "$pid"
else
    echo "No matching Python3 process found."
fi
