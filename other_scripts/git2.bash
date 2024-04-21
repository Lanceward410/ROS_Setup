#!/bin/bash

function pairkey() {
  echo "Press Enter to continue to key pairing"
  read -r
  echo "Carrying on,"
  cd ~/ROS_Setup
  eval "$(ssh-agent -s)"
  ssh-add ~/.ssh/$keyfilename
  cat ~/.ssh/$keyfilename.pub | pbcopy
  ssh -T git@github.com
  git remote set-url origin git@github.com:Lanceward410/ROS_Setup.git
}

git config --global user.email "lanceward410@gmail.com"
git config --global user.name "lanceward410"

# Prompt the user to define a filename for the SSH key
read -p "Enter the filename for the SSH key (without extension): " keyfilename

# Generate SSH key with the specified filename
ssh-keygen -t rsa -b 4096 -C "lanceward410@gmail.com" -f ~/.ssh/"$keyfilename"

# Check if an SSH key already exists with the specified filename
if [ -f ~/.ssh/"$keyfilename" ]; then
  echo "SSH key already exists at ~/.ssh/$keyfilename."
  echo "If you proceed, the existing key will be overwritten."
  read -p "Do you want to continue? (y/n): " choice
  if [[ "$choice" != "y" ]]; then
    echo "SSH key generation aborted."
    exit 0
  fi
fi

# Concatenate key to easy-to-find file, and display it to user
if [ -f ~/"Github_SSH_key.txt" ]; then
  echo "SSH key already exists at ~/Github_SSH_key.txt."
  echo "If you proceed, the existing key will be overwritten."
  read -p "Do you want to continue? (y/n): " choice
  if [[ "$choice" != "y" ]]; then
    echo "Concatenation process aborted"
    pairkey # Pairs key anyway, you just won't have it saved at ~/Github_SSH_key.txt
    exit 1
  fi
fi
echo "Your SSH public key is:"
echo "Your SSH public key is:" >> ~/Github_SSH_key.txt
cat ~/.ssh/"$keyfilename.pub"
cat ~/.ssh/"$keyfilename.pub" >> ~/Github_SSH_key.txt

# Instructions for adding the public key to GitHub
echo "Please request that your public key get added to the main"
echo "Github repository at: https://github.com/settings/keys"
echo "You won't lose this key! It has been stored in ~/Github_SSH_key.txt"

# Continue to pair your SSH
pairkey