#!/bin/bash

if [[ $# -eq 0 ]]; then
    echo "No flag provided. Please specify your user key to sign in." >&2
    echo "example:   linuxuser:~$   setuser -lance"
    exit 1
fi

setuser() {
    case $1 in
        -lance789)
            echo "user lance selected"
            git config --global user.email "lanceward410@gmail.com"
            git config --global user.name "lanceward410"
            ;;
        -chris)
            echo "chris not setup yet"
            ;;
        -danny)
            echo "danny not setup yet"
            ;;
        *)
            echo "no valid user selected"
            ;;
    esac
}

setuser "$1"
