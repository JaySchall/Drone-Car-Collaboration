#!/bin/bash
echo "Running iper3 server script..."
iperf3 -s -p 5201 -V --logfile iperf3_TCP_Test_server.txt
#note that iperf3 appends to the log file rather than overwrite it.
