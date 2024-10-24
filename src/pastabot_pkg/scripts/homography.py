import numpy as np
import cv2

SHORT_SIDE = 0.8
LONG_SIDE = 1.5

short_size_px = 416
long_size_px =780

world_points = np.array([
    (0, -SHORT_SIDE/2),            # A 
    (0, SHORT_SIDE/2),             # B
    (LONG_SIDE, -SHORT_SIDE/2),    # D
    (LONG_SIDE, SHORT_SIDE/2)      # C
    ])

img_points = np.array([
    (0,long_size_px),
    (short_size_px,long_size_px),
    (0, 0),
    (short_size_px,0)
    ])

H, status = cv2.findHomography(img_points, world_points)
if status is None or not status.all():
    print("Warning: some points did not result in a valid homography.")

H = H / H[2, 2]
np.save("src/pastabot_pkg/scripts/homography_matrix.npy", H)

