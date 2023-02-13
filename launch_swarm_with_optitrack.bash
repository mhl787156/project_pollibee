#!/bin/bash

# bash script to run the program
# Arguments
drone_namespaces=('cf0' 'cf1')
drone_uris=("radio://1/80/250K/E7E7E7E702" "radio://0/70/250K/E7E7E7E701")

# Run the program with index 0
./launch_as2.bash "${drone_namespaces[0]}" "${drone_uris[0]}" true &
# Run the program with index 1
./launch_as2.bash "${drone_namespaces[1]}" "${drone_uris[1]}"  &

#tmux attach -t :0
