#!/bin/bash

  git config --global user.email "lanceward410@gmail.com"
  git config --global user.name "lanceward410"

# Check if an SSH key already exists
if [ -f ~/.ssh/id_rsa ]; then
  echo "SSH key already exists at ~/.ssh/id_rsa."
  echo "If you proceed, the existing key will be overwritten."
  read -p "Do you want to continue? (y/n): " choice
  if [[ "$choice" != "y" ]]; then
    echo "SSH key generation aborted."
    exit 0
  fi
fi

# Generate SSH key
ssh-keygen -t rsa -b 4096 -C "lanceward410@gmail.com"

# Display the public key
echo "Your SSH public key is:"
cat ~/.ssh/id_rsa.pub

# Instructions for adding the public key to GitHub
echo "Add this public key to your GitHub account: https://github.com/settings/keys"
