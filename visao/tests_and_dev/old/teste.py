import os
import glob

import dlib

# Path to the video frames
video_folder = os.path.join("..", "examples", "video_frames")

# Create the correlation tracker - the object needs to be initialized
# before it can be used
tracker = dlib.correlation_tracker()

win = dlib.image_window()
# We will track the frames as we load them off of disk
for k, f in enumerate(sorted(glob.glob(os.path.join(video_folder, "*.jpg")))):
    print("Processing Frame {}".format(k))
    img = dlib.load_rgb_image(f)
    # We need to initialize the tracker on the first frame
    if k == 0:
        # Start a track on the juice box. If you look at the first frame you
        # will see that the juice box is contained within the bounding
        # box (74, 67, 112, 153).abs
        tracker.start_track(img, dlib.rectangle(74, 67, 112, 153))
    else:
        # Else we just attempt to track from the previous frame
        tracker.update(img)
        win.clear_overlay()
        win.set_image(img)
        win.add_overlay(tracker.get_position())
        dlib.hit_enter_to_continue()
