#!/bin/bash
echo "Running iperf3 client script for 30 seconds..."
iperf3 -V -c 192.168.11.1 -t 45 --logfile iperf3_TCP_Test_client.txt &
iperf_pid=$!

for i in {1..45}; do
  echo "$i seconds passed."
  sleep 1
done

# Kill the iperf3 process
kill $iperf_pid
wait $iperf_pid 2>/dev/null
echo "iperf3 client script completed."
