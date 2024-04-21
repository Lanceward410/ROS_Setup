#!/bin/bash

sudo apt update
sudo apt upgrade -y
sudo apt-get install nvidia-driver-535 -y
sudo apt install prime
sudo prime-select nvidia