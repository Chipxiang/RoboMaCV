#!/bin/bash
stty -F /dev/ttySAC0 sane > PortSetup
stty -F /dev/ttySAC0 speed 115200 >> PortSetup
stty -F /dev/ttySAC0 -a >> PortSetup
echo "Setup complete!"

