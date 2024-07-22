import tkinter as tk
from tkinter import messagebox, simpledialog
import subprocess
import os
import signal
import sys
import threading

# Function to run a bash script
def run_script(script_name):
    script_path = os.path.expanduser('~/ROS_Setup/scripts/{}'.format(script_name))
    if not os.path.exists(script_path):
        messagebox.showerror("Error", "Script not found: {}".format(script_path))
        return

    password = simpledialog.askstring("Password", "Enter your sudo password:", show='*')
    if not password:
        messagebox.showwarning("Warning", "No password entered. Operation cancelled.")
        return

    def stream_output(process):
        while True:
            output = process.stdout.readline()
            if output == '' and process.poll() is not None:
                break
            if output:
                text_widget.configure(state='normal')
                text_widget.insert(tk.END, output)
                text_widget.configure(state='disabled')
                text_widget.see(tk.END)
        err = process.stderr.read()
        if err:
            text_widget.configure(state='normal')
            text_widget.insert(tk.END, "\nError:\n" + err)
            text_widget.configure(state='disabled')
        process.communicate()

    try:
        command = f"echo {password} | sudo -S bash {script_path}"
        process = subprocess.Popen(command, shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE, universal_newlines=True)
        threading.Thread(target=stream_output, args=(process,), daemon=True).start()
    except Exception as e:
        messagebox.showerror("Error", str(e))

# Signal handler for graceful exit
def signal_handler(sig, frame):
    print('Interrupt received, shutting down...')
    root.destroy()
    sys.exit(0)

# Register the signal handler
signal.signal(signal.SIGINT, signal_handler)

# Create the main window
root = tk.Tk()
root.title("ROS Setup GUI")

# Add padding around buttons
button_padding = {'padx': 10, 'pady': 5}

# Main Buttons Section
main_label = tk.Label(root, text="Main Installers", font=('Helvetica', 14, 'bold'))
main_label.pack(pady=(10, 5))

# Text widget to show terminal output
text_widget = tk.Text(root, state='disabled', width=80, height=20)
text_widget.pack(pady=10)

main_buttons = [
    ('ROS1 Installer', 'ros1_installer.sh'),
    ('ROS2 Installer', 'ros2_installer.sh'),
    ('Repair Software', 'repair_software.sh')
]

for text, script in main_buttons:
    button = tk.Button(root, text=text, command=lambda s=script: run_script(s))
    button.pack(**button_padding)

# Divider
divider1 = tk.Label(root, text="-"*50)
divider1.pack(pady=(10, 10))

# Secondary Buttons Section
secondary_label = tk.Label(root, text="Setup Scripts", font=('Helvetica', 14, 'bold'))
secondary_label.pack(pady=(10, 5))

secondary_buttons = [
    ('Setup Git Keys', 'git2.bash'),
    ('Setup Git User', 'gituser.bash'),
    ('Setup Nvidia Drivers', 'nvidia_setup.bash'),
    ('Repair Sound', 'sound.bash')
]

for text, script in secondary_buttons:
    button = tk.Button(root, text=text, command=lambda s=script: run_script(s))
    button.pack(**button_padding)

# Divider
divider2 = tk.Label(root, text="-"*50)
divider2.pack(pady=(10, 10))

# Readme Scripts Section
readme_label = tk.Label(root, text="Readme Scripts", font=('Helvetica', 14, 'bold'))
readme_label.pack(pady=(10, 5))

# Example readme scripts, you can add more as needed
readme_buttons = [
    ('Scout Mini README', 'readme1.bash'),
    ('AgileX LIMO README', 'readme2.bash'),
    ('Yahboom Rosmaster README', 'readme3.bash'),
    ('Yahboom MicroROS README', 'readme4.bash')
]

for text, script in readme_buttons:
    button = tk.Button(root, text=text, command=lambda s=script: run_script(s))
    button.pack(**button_padding)

# Start the GUI loop
root.mainloop()
