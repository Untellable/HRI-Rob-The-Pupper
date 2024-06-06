import tkinter as tk
from tkinter import messagebox, PhotoImage
from PIL import Image, ImageTk
import cv2
import os
print(os.getcwd())
from feedback.feedback import get_feedback, color_dict_HSV

from gtts import gTTS
import audio2numpy
import sounddevice as sd
import os
import sys
import time

count = 0

def toggle_fullscreen(event=None):
    global is_fullscreen
    is_fullscreen = not is_fullscreen  # Toggle state
    root.attributes("-fullscreen", is_fullscreen)

def end_fullscreen(event=None):
    global is_fullscreen
    is_fullscreen = False
    root.attributes("-fullscreen", False)

def start_puzzle():
    global mask_image
    if not level.get():
        messagebox.showinfo("Select Level", "Please select a difficulty level!")
        return

    image_path = f"test1.jpg"
    try:
        cv_img = cv2.imread(image_path)
        if cv_img is None:
            raise IOError("Image file not found")
        mask_image = cv_img.copy()
        resized_cv_img = cv2.resize(cv_img, ( (1280//2, 720//2)))  # Resize the image
        cv_img_rgb = cv2.cvtColor(resized_cv_img, cv2.COLOR_BGR2RGB)
        pil_img = Image.fromarray(cv_img_rgb)
        img = ImageTk.PhotoImage(pil_img)
        image_label.config(image=img)
        image_label.image = img
    except IOError as e:
        messagebox.showerror("Image Load Error", str(e))
        return

    mask_level_label.config(text=f"Mask Level: {level.get()}")

def generate_feedback():
    global curr_image
    global mask_image
    if not level.get():
        messagebox.showinfo("Select Level and start puzzle", "Please select a difficulty level and start puzzle!")
        return

    global count  # Declare count as global to modify it
    count += 1  # Increment count each time feedback is generated
    update_puzzle_image()

    color_name = "green"
    color = color_dict_HSV[color_name]
    # im2 = cv2.imread("test2.jpg")
    # mask = cv2.imread(f"1_mask_{color_name}.png", cv2.IMREAD_UNCHANGED)
    img_feedback = get_feedback(curr_image, mask_image, hidden_quads=[0, 1, 2, 3], color_lower=color[1], color_upper=color[0])

    feedback_message = f"Check: {count} - Feedback: {img_feedback}"

    gTTS_obj = gTTS(text = img_feedback, lang='en')
    gTTS_obj.save('feedback.mp3')

    x, sr = audio2numpy.audio_from_file('feedback.mp3')
    sd.play(x, sr)
    update_feedback(feedback_message)
    update_feedback(feedback_message)


def update_puzzle_image():
    global image_label
    global curr_image
    try:
        os.system('rsync ubuntu@' + ip + ':/home/ubuntu/transfer_dir/camera_image.png camera_frame.png')
        image_path = "camera_frame.png"
        cv_img = cv2.imread(image_path)
        curr_image = cv_img.copy()
        if cv_img is None:
            raise IOError("Image file not found")
        resized_cv_img = cv2.resize(cv_img,( (1280//2, 720//2)))  # Resize the image
        cv_img_rgb = cv2.cvtColor(resized_cv_img, cv2.COLOR_BGR2RGB)
        pil_img = Image.fromarray(cv_img_rgb)
        img = ImageTk.PhotoImage(pil_img)
        image_label.configure(image=img)
        image_label.image = img
    except:
        os.system('rsync ubuntu@' + ip + ':/home/ubuntu/transfer_dir/camera_image.png camera_frame.png')
        image_path = "camera_frame.png"
        cv_img = cv2.imread(image_path)
        curr_image = cv_img.copy()
        if cv_img is None:
            raise IOError("Image file not found")
        resized_cv_img = cv2.resize(cv_img, (1280//2, 720//2))  # Resize the image
        cv_img_rgb = cv2.cvtColor(resized_cv_img, cv2.COLOR_BGR2RGB)
        pil_img = Image.fromarray(cv_img_rgb)
        img = ImageTk.PhotoImage(pil_img)
        image_label.configure(image=img)
        image_label.image = img




def update_feedback(message):
    feedback_text.config(state=tk.NORMAL)
    feedback_text.delete('1.0', tk.END)
    feedback_text.insert(tk.END, message)
    feedback_text.config(state=tk.DISABLED)

def set_level(selected_level):
    level.set(selected_level)
    for button, lvl in zip([easy_button, medium_button, hard_button], ["Easy", "Medium", "Hard"]):
        button.config(relief=tk.SUNKEN if level.get() == lvl else tk.RAISED)

def quit_application():
    root.destroy()

def key_press(event):
    key = event.keysym

    if key in {'w', 'a', 's', 'd', 'z', 'c', 'x', 'f'}:
        print(f"Move: {key}")
        time_now = time.time()
        #save it to file key.txt
        with open('key.txt', 'w') as f:
            f.write(str(time_now) + '\n')
            f.write(key)

        #rsync to pupper
        os.system('rsync key.txt ubuntu@' + ip + ':/home/ubuntu/transfer_dir/key.txt')


class mainWindow():
    def __init__(self, window, image_label, ip):
        self.window = window
        self.image_label = image_label
        self.ip = ip
        self.image_path = "frame.jpg"

        self.set_image()

    def set_image(self):
        try:
            os.system('rsync ubuntu@' + self.ip + ':/home/ubuntu/transfer_dir/camera_image.png camera_frame.png')
            image = Image.open(image_path)

            #masking code
            img = ImageTk.PhotoImage(image)
            self.image_label.configure(image=img)
            self.image_label.image = img
            self.window.update()
        except:
            self.window.after(10, self.set_image)

        self.window.after(1000, self.set_image)

if __name__ == "__main__":

    global image_label
    ip = sys.argv[1]
    # Main window
    root = tk.Tk()
    root.title("Rob the Builder")
    root.geometry('1000x600')

    is_fullscreen = True
    root.attributes("-fullscreen", is_fullscreen)
    root.bind("<F11>", toggle_fullscreen)
    root.bind("<Escape>", end_fullscreen)

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

    feedback_button = tk.Button(root, text="Generate Feedback", command=generate_feedback)
    feedback_button.pack(pady=10)

    mask_level_label = tk.Label(root, text="Mask Level: Not Selected", font=("Arial", 16))
    mask_level_label.pack(pady=5)

    container = tk.Frame(root)
    container.pack(expand=True, fill=tk.BOTH, padx=20, pady=10)

    image_label = tk.Label(container, height=1280, width=720)
    image_label.pack(side=tk.LEFT, padx=10)

    feedback_text = tk.Text(container, height=20, width=50, font=("Arial", 16))
    feedback_text.pack(side=tk.RIGHT, padx=10)
    feedback_text.config(state=tk.DISABLED)

    try:
        os.system('rsync ubuntu@' + ip + ':/home/ubuntu/transfer_dir/camera_image.png camera_frame.png')
        image_path = "camera_frame.png"
        image = Image.open(image_path)
        img = ImageTk.PhotoImage(image)
        # image_label = tk.Label(root, image=img)
        # image_label.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
    except:
        os.system('rsync ubuntu@' + ip + ':/home/ubuntu/transfer_dir/camera_image.png camera_frame.png')
        image_path = "camera_frame.png"
        image = Image.open(image_path)
        img = ImageTk.PhotoImage(image)
        # image_label = tk.Label(root, image=img)
        # image_label.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)


    # mainWindow(root, image_label, ip)

    root.mainloop()
