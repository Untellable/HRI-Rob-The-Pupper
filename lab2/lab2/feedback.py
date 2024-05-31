import cv2
import numpy as np

# When calculating the percent of pixels matching between two images, 
# use these thresholds and labels to describe how close they are
similarity_thresholds = np.array([.6, .8, .9])
similarity_threshold_labels = ["not similar", "somewhat similar", "similar", "very similar", "matching"]

quadrant_names = ["top left", "top right", "bottom left", "bottom right"]

# Convert similarity percent into string descriptor based on thresholds
def threshold_text(perc_similar):
    thresholds_passed = np.sum(perc_similar > similarity_thresholds)
    similarity_description = similarity_threshold_labels[thresholds_passed]
    return similarity_description

# Calculate percent shared pixels for each quadrant pair between 2 images
def compare_quads(im1, im2):
    # Expects images be square and same size
    side_len = im1.shape[0]
    half_len = side_len // 2
    assert side_len == im2.shape[0]
    comp_results = []
    for row in [0,half_len]:
        for col in [0,half_len]:
            comp_results.append(np.mean(im1[row:row + half_len] == im2[col:col + half_len]))
    return comp_results

# Translate an image 
# diff: int tuple pair on number of pixels to move horizontally and vertically.
def move_image(im, diff: tuple[int, int]):
    pad = max(abs(diff[0]), abs(diff[1]))
    padded_im = np.pad(im, pad, constant_values = -1)
    (im_x, im_y) = im.shape
    return padded_im[pad - diff[0]: pad + im_x - diff[0], pad - diff[1]: pad + im_y - diff[1]]

# Translate move (numpy array 2 int elements) into a readable text version
def explain_move(move, im_shape):
    move_dirs = move > 0
    x_names = ["left", "right"]
    y_names = ["up", "down"]

    # Calculate distance to move as percent of image dimensions
    x_perc = np.rint(100 * abs(move[0] / im_shape[0])).astype(int)
    y_perc = np.rint(100 * abs(move[1] / im_shape[1])).astype(int)

    return f"{x_perc}% {x_names[move_dirs[0]]} and {y_perc}% {y_names[move_dirs[1]]}"
    

# Initiate ORB detector
orb = cv2.ORB_create()

# Comparing two images, look for keypoint pairs between them
# Test if moving the first image to match keypoint pairs would make any of the 
# image quadrants match. Only checks for translations, not rotations
# n is then number of keypoint pair translations to test
# NOTE: matching performs badly with ~<200x200 images
def check_close_quad(im1, im2, n = 3):
    # find the keypoints and descriptors with ORB
    # Method taken from https://docs.opencv.org/4.x/dc/dc3/tutorial_py_matcher.html
    kp1, des1 = orb.detectAndCompute(im1,None)
    kp2, des2 = orb.detectAndCompute(im2,None)

    # create BFMatcher object
    bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
     
    # Match descriptors.
    matches = bf.match(des1,des2)
     
    # Sort them in the order of their distance.
    matches = sorted(matches, key = lambda x:x.distance)

    top_match_score = 0
    top_match_quad = 0
    top_match_move = (0,0)
    for i in range(max(n, len(matches))):
        # Calculate keypoint distance between images for one match
        kp1Idx = matches[i].queryIdx
        kp2Idx = matches[i].trainIdx
        move = np.rint(kp2[kp2Idx].pt).astype(int) - np.rint(kp1[kp1Idx].pt).astype(int)
        quad_scores = compare_quads(move_image(im1, move), im2)
        # Calc best quad match
        for quad in range(4):
            if quad_scores[quad] > top_match_score:
                top_score = quad_scores[quad]
                top_quad = quad
                top_move = move
    move_text = explain_move(top_move, im1.shape)
    return f"I detect a {threshold_perc(top_score)} match for the {quadrant_names[top_quad]} quadrant if you move {top_move}"
