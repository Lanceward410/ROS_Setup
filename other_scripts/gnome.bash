#!/bin/bash

# Open a new terminal window and list directory contents
gnome-terminal -- bash -c "ls; exec bash"

# Open a new terminal window and run top command
gnome-terminal -- bash -c "top"

# Open a new terminal window and start Python interactive shell
gnome-terminal -- bash -c "python; exec bash"