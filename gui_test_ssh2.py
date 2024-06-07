import tkinter as tk
from tkinter import messagebox, PhotoImage
from PIL import Image, ImageTk
import cv2
from feedback.feedback import get_feedback, color_dict_HSV, camera_to_mask
from time import time

from gtts import gTTS
#import audio2numpy
#import sounddevice as sd
import os
import sys
import numpy as np
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
    global mask_image, hidden_quadrants, curr_image
    update_puzzle_image()
    hidden_quadrants = [0, 1, 2, 3]
    if not level.get():
        messagebox.showinfo("Select Level", "Please select a difficulty level!")
        return

    image_path = f"mask_true.png"
    try:
        cv_img = cv2.imread(image_path)
        if cv_img is None:
            raise IOError("Image file not found")
        mask_image = cv_img.copy()
        color_name = "blue"
        color = color_dict_HSV[color_name]
        mask_image = camera_to_mask(mask_image, color_lower = color[1], color_upper = color[0])
    except IOError as e:
        messagebox.showerror("Image Load Error", str(e))
        return
    
    update_image()

    mask_level_label.config(text=f"Mask Level: {level.get()}")

def generate_feedback():
    global curr_image, mask_image, count, hidden_quadrants
    t = time.time()
    if not level.get():
        messagebox.showinfo("Select Level and start puzzle", "Please select a difficulty level and start puzzle!")
        return

    count += 1  # Increment count each time feedback is generated
    update_puzzle_image()
    print(f"update_puzzle_image time: {time.time() - t}")
    t = time.time()
    color_name = "blue"
    color = color_dict_HSV[color_name]
    # im2 = cv2.imread("test2.jpg")
    # mask = cv2.imread(f"1_mask_{color_name}.png", cv2.IMREAD_UNCHANGED)
    img_feedback, hidden_quadrants = get_feedback(curr_image, mask_image, hidden_quads=hidden_quadrants, color_lower=color[1], color_upper=color[0])
    print(f"get_feedback time: {time.time() - t}")
    t = time.time()
    # if 'uncovered the' in img_feedback:
    #     hidden_quadrants.remove(int(img_feedback.split(' ')[-2]))
    # elif 'solution' in img_feedback:
    #     hidden_quadrants = []
    #     print('hi')

    update_image()
    print(f"update_image time: {time.time() - t}")
    t = time.time()
    feedback_message = f"Check: {count} - Feedback: {img_feedback}"

    # gTTS_obj = gTTS(text = img_feedback, lang='en')
    # gTTS_obj.save('feedback.mp3')

    # os.system('rsync feedback.mp3 ubuntu@' + ip + ':/home/ubuntu/transfer_dir/feedback.mp3')

    time_now = time.time()
    #save it to file key.txt
    with open('key.txt', 'w') as f:
        f.write(str(time_now) + '\n')
        f.write('f')

    #rsync to pupper
    os.system('rsync key.txt ubuntu@' + ip + ':/home/ubuntu/transfer_dir/key.txt')


    update_feedback(feedback_message)
    print(f"final generate_feedback time: {time.time() - t}")


def update_puzzle_image():
    global image_label, curr_image
    try:
        os.system('rsync ubuntu@' + ip + ':/home/ubuntu/transfer_dir/camera_image.png camera_frame.png')
        time.sleep(0.2)
        image_path = "camera_frame.png"
        cv_img = cv2.imread(image_path)
        curr_image = cv_img.copy()
        if cv_img is None:
            raise IOError("Image file not found")

    except:
        time.sleep(0.2)
        os.system('rsync ubuntu@' + ip + ':/home/ubuntu/transfer_dir/camera_image.png camera_frame.png')
        image_path = "camera_frame.png"
        cv_img = cv2.imread(image_path)
        curr_image = cv_img.copy()
        if cv_img is None:
            raise IOError("Image file not found")
        # resized_cv_img = cv2.resize(cv_img, (1280//2, 720//2))  # Resize the image
        # cv_img_rgb = cv2.cvtColor(resized_cv_img, cv2.COLOR_BGR2RGB)
        # pil_img = Image.fromarray(cv_img_rgb)
        # img = ImageTk.PhotoImage(pil_img)
        # image_label.configure(image=img)
        # image_label.image = img

def update_image():
    global curr_image, image_label, mask_image, hidden_quadrants
    quad_flags = [True, True, True, True]
    for quad in hidden_quadrants:
        quad_flags[quad] = False
    masked_image = mask_quadrants(curr_image, mask_image, quad_flags=quad_flags, alpha=0.2)
    resized_cv_img = cv2.resize(masked_image,( (1280//2, 720//2)))  # Resize the image
    cv_img_rgb = cv2.cvtColor(resized_cv_img, cv2.COLOR_BGR2RGB)
    pil_img = Image.fromarray(cv_img_rgb)
    img = ImageTk.PhotoImage(pil_img)
    image_label.configure(image=img)
    image_label.image = img

def mask_quadrants(live_image, mask_image, quad_flags = [True, True, True, True], alpha = 0.5):
    assert live_image.shape[0] == mask_image.shape[0] and live_image.shape[1] == mask_image.shape[1]
    height, width = mask_image.shape

    result = np.float32(np.copy(live_image))
    mask_image = np.float32(mask_image)/255

    #scale from 0-1 to alpha-1
    mask_image = (1 - alpha*2) * mask_image + alpha*2

    mask_image = np.repeat(mask_image[:, :, np.newaxis], 3, axis=2)

    zeros_image = np.ones_like(mask_image)*alpha

    for i in range(2):
        for j in range(2):
            if quad_flags[i * 2 + j]:
                result[i * height // 2: (i + 1) * height // 2, j * width // 2: (j + 1) * width // 2] *= mask_image[i * height // 2: (i + 1) * height // 2, j * width // 2: (j + 1) * width // 2]
            else:
                result[i * height // 2: (i + 1) * height // 2, j * width // 2: (j + 1) * width // 2] *= zeros_image[i * height // 2: (i + 1) * height // 2, j * width // 2: (j + 1) * width // 2]

    return np.uint8(result)


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

    if key in {'w', 'a', 's', 'd', 'z', 'c', 'x', 'f', 'q', 'e'}:
        print(f"Move: {key}")
        time_now = time.time()
        #save it to file key.txt
        with open('key.txt', 'w') as f:
            f.write(str(time_now) + '\n')
            f.write(key)

        #rsync to pupper
        os.system('rsync key.txt ubuntu@' + ip + ':/home/ubuntu/transfer_dir/key.txt')


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
