#!/bin/bash

# Change to the ROS_Setup directory
cd ~/ROS_Setup

# Check if python3 is installed
if ! dpkg-query -W -f='${Status}' python3 2>/dev/null | grep -q "ok installed"; then
    echo "python3 is not installed. Installing now..."
    sudo apt-get install python3 -y
else
    echo "Python3 is already installed."
fi

# Check if python3-pip is installed
if ! dpkg-query -W -f='${Status}' python3-pip 2>/dev/null | grep -q "ok installed"; then
    echo "python3-pip is not installed. Installing now..."
    sudo apt-get install python3-pip -y
else
    echo "python3-pip is already installed."
fi

# Install pyqt5 using pip
pip3 install pyqt5

# Check if python3-tk is installed
if ! dpkg-query -W -f='${Status}' python3-tk 2>/dev/null | grep -q "ok installed"; then
    echo "python3-tk is not installed. Installing now..."
    sudo apt-get install python3-tk -y
else
    echo "Python3-tk is already installed."
fi

# Create a .desktop file
DESKTOP_FILE=~/.local/share/applications/ros_setup_gui.desktop
cat > $DESKTOP_FILE <<EOL
[Desktop Entry]
Version=1.0
Name=ROS Setup GUI
Comment=Launch the ROS Setup GUI
Exec=/home/$USER/ROS_Setup/launchgui.sh
Icon=/home/$USER/ROS_Setup/photo-delicious-hamburger-isolated-white-background.png
Terminal=false
Type=Application
Categories=Utility;Application;
EOL

# Make the .desktop file executable
chmod +x $DESKTOP_FILE

# Rebuild desktop database
update-desktop-database ~/.local/share/applications/

# Make the launch script executable
chmod +x ~/ROS_Setup/launchgui.sh

# Notify the user
echo "ROS Setup GUI has been added to your applications. You can now find it in the dashboard and add it to your hotbar."

