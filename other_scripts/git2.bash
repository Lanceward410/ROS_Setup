#!/bin/bash

git config --global user.email "lanceward410@gmail.com"
git config --global user.name "lanceward410"

# Prompt the user to define a filename for the SSH key
read -p "Enter the filename for the SSH key (without extension): " filename

# Generate SSH key with the specified filename
ssh-keygen -t rsa -b 4096 -C "lanceward410@gmail.com" -f ~/.ssh/"$filename"

# Check if an SSH key already exists with the specified filename
if [ -f ~/.ssh/"$filename" ]; then
  echo "SSH key already exists at ~/.ssh/$filename."
  echo "If you proceed, the existing key will be overwritten."
  read -p "Do you want to continue? (y/n): " choice
  if [[ "$choice" != "y" ]]; then
    echo "SSH key generation aborted."
    exit 0
  fi
fi

# Display the public key
echo "Your SSH public key is:"
echo "Your SSH public key is:" >> ~/Github_SSH_key.txt
cat ~/.ssh/"$filename.pub"
cat ~/.ssh/"$filename.pub" >> ~/Github_SSH_key.txt

# Instructions for adding the public key to GitHub
echo "Add this public key to your GitHub account: https://github.com/settings/keys"
echo "You won't lose this key! It has been stored in ~/Github_SSH_key.txt"
