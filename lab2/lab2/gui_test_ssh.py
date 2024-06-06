import tkinter as tk
from tkinter import messagebox, PhotoImage
from PIL import Image, ImageTk

import os
import sys
import time

def start_puzzle():
    if not level.get():
        messagebox.showinfo("Select Level", "Please select a difficulty level!")
        return

    global feedback_text
    # Create a new window
    puzzle_window = tk.Toplevel(root)
    puzzle_window.title(f"Puzzle {level.get()}")
    print('hi')
    # Image mask (use an actual image file path in the same folder)
    # image_path = "mask_medium.PNG"  # Ensure 'your_image.png' is in the same folder as your script
    image_path = f"mask_{level.get().lower()}.png"
    img = PhotoImage(file=image_path)
    image_label = tk.Label(puzzle_window, image=img)
    image_label.image = img  # Keep a reference!
    image_label.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

    # Feedback text box
    feedback_text = tk.Text(puzzle_window, height=10, width=30)
    feedback_text.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True)
    # feedback_text.insert(tk.END, "Feedback will appear here.")
    update_feedback("Puzzle started! Good luck.")

def update_feedback(message):
    """Update the feedback text box with a new message."""
    if feedback_text:
        feedback_text.delete('1.0', tk.END)  # Clear existing text
        feedback_text.insert(tk.END, message)  # Insert new message

def set_level(selected_level):
    level.set(selected_level)
    # Reset all buttons to RAISED
    easy_button.config(relief=tk.RAISED)
    medium_button.config(relief=tk.RAISED)
    hard_button.config(relief=tk.RAISED)
    # Set the selected button to SUNKEN
    if selected_level == "Easy":
        easy_button.config(relief=tk.SUNKEN)
    elif selected_level == "Medium":
        medium_button.config(relief=tk.SUNKEN)
    elif selected_level == "Hard":
        hard_button.config(relief=tk.SUNKEN)

def quit_application():
    root.destroy()

def key_press(event):
    key = event.keysym

    if key in {'f'}:
        print(f"Move: {key}")
        time_now = time.time()
        #save it to file key.txt
        with open('key.txt', 'w') as f:
            f.write(str(time_now) + '\n')
            f.write(key)

        #scp to pupper
        os.system('scp key.txt ubuntu@' + ip + ':/home/ubuntu/transfer_dir/key.txt')


class mainWindow():
    def __init__(self, window, image_label, ip):
        self.window = window
        self.image_label = image_label
        self.ip = ip
        self.image_path = "frame.jpg"

        self.set_image()

    def set_image(self):
        try:
            os.system('scp ubuntu@' + self.ip + ':/home/ubuntu/transfer_dir/camera_image.png camera_frame.png')
            image = Image.open(image_path)

            #masking code
            img = ImageTk.PhotoImage(image)
            self.image_label.configure(image=img)
            self.image_label.image = img
            self.window.update()
        except:
            self.window.after(50, self.set_image)

        self.window.after(500, self.set_image)

if __name__ == "__main__":
    ip = sys.argv[1]
    # Main window
    root = tk.Tk()
    root.title("Rob the Builder")

    # Heading
    heading_label = tk.Label(root, text="Rob the Builder", font=("Arial", 24))
    heading_label.pack(pady=20)

    # Difficulty level options
    level = tk.StringVar()  # To store the selected difficulty level
    frame = tk.Frame(root)
    frame.pack(pady=20)

    #track key press
    root.bind("<Key>", key_press)

    easy_button = tk.Button(frame, text="Easy", command=lambda: set_level("Easy"))
    easy_button.pack(side=tk.LEFT)

    medium_button = tk.Button(frame, text="Medium", command=lambda: set_level("Medium"))
    medium_button.pack(side=tk.LEFT)

    hard_button = tk.Button(frame, text="Hard", command=lambda: set_level("Hard"))
    hard_button.pack(side=tk.LEFT)

    # Start button
    start_button = tk.Button(root, text="Start the Puzzle", command=start_puzzle)
    start_button.pack(pady=20)

    # Quit button
    quit_button = tk.Button(root, text="Quit", command=quit_application)
    quit_button.pack(side=tk.RIGHT, pady=20, padx=10)

    try:
        os.system('scp ubuntu@' + ip + ':/home/ubuntu/transfer_dir/camera_image.png camera_frame.png')
        image_path = "camera_frame.png"
        image = Image.open(image_path)
        img = ImageTk.PhotoImage(image)
        image_label = tk.Label(root, image=img)
        image_label.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
    except:
        os.system('scp ubuntu@' + ip + ':/home/ubuntu/transfer_dir/camera_image.png camera_frame.png')
        image_path = "camera_frame.png"
        image = Image.open(image_path)
        img = ImageTk.PhotoImage(image)
        image_label = tk.Label(root, image=img)
        image_label.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)


    mainWindow(root, image_label, ip)

    root.mainloop()
