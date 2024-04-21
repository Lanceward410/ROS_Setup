#!/bin/bash

sudo apt update
sudo apt upgrade -y
sudo apt-get install nvidia-driver-535 nvidia-prime -y
sudo prime-select nvidia