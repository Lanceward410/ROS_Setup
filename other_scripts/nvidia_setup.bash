#!/bin/bash

flag=$1

nvidia_setup() {
    sudo apt update
    sudo apt upgrade -y
    sudo add-apt-repository ppa:graphics-drivers/ppa -y
    echo -ne '\n'
    sudo apt-get update
    sudo apt install nvidia-prime -y
    sudo apt-get install nvidia-driver-535 nvidia-prime -y
        case $flag in
            -n)
                echo "flag to prime-select nvidia set to ON"
                sudo prime-select nvidia
                ;;
            -i)
                echo "flag to prime-select intel set to ON"
                sudo prime-select intel
                ;;
            -o)
                echo "flag to prime-select on-demand set to ON"
                sudo prime-select on-demand
                ;;
            *)
                echo "flag to use prime-select either set to off or invalid"
                echo "Below is your current prime selection:"
                prime-select query
                ;;
        esac
}

nvidia_setup "$flag"
