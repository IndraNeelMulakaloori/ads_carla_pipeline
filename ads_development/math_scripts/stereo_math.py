import numpy as np 
import math
import cv2 as cv 


def stereo_math_helper(left_img_array, right_img_array, baseline, focal_length, cx, cy):
    ### convert images to grayscale
    left_img_grayscale = cv.cvtColor(left_img_array, cv.COLOR_RGB2GRAY)
    right_img_grayscale = cv.cvtColor(right_img_array, cv.COLOR_RGB2GRAY)

    ### compute dosaprtiy 
    block_size = 5  # Size of the matching window (must be odd). 5 or 7 is standard.
    
    stereo = cv.StereoSGBM_create(
        minDisparity=0,
        numDisparities=128,      # Must be divisible by 16. Higher = can see closer objects.
        blockSize=block_size,
        
        # P1 and P2 enforce smoothness. Crucial for textureless CARLA roads.
        P1=8 * 1 * block_size**2,
        P2=32 * 1 * block_size**2,
        
        disp12MaxDiff=1,         # Max allowed difference in the left-right consistency check
        uniquenessRatio=15,      # Margin by which the best match must beat the second best
        speckleWindowSize=100,   # Filters out isolated noise patches
        speckleRange=32          # Max disparity variation within a speckle
    )

    # Compute the Disparity Map
    raw_disparity = stereo.compute(left_img_grayscale, right_img_grayscale)

    # 
    # OpenCV internally multiplies disparity by 16 to store sub-pixel precision in a 16-bit integer.
    # You MUST divide by 16.0 to get the true pixel shift.
    disparity = raw_disparity.astype(np.float32) / 16.0

    ## reprojection 
    # Get image dimensions
    h, w = disparity.shape

    # Create pixel coordinate grids (u = columns/width, v = rows/height)
    u, v = np.meshgrid(np.arange(w), np.arange(h))

    # Safe Z calculation: only divide where disparity > 0
    valid_disp = disparity > 0
    Z_opt = np.zeros_like(disparity, dtype=np.float32)
    Z_opt[valid_disp] = (focal_length * baseline) / disparity[valid_disp]

    # Vectorized reprojection for X and Y
    X_opt = (u - cx) * Z_opt / focal_length
    Y_opt = (v - cy) * Z_opt / focal_length

    #  Mask out bad geometry and limit physical range (e.g., 50 meters max)
    valid_mask = (Z_opt > 0) & (Z_opt < 50.0)

    # Apply mask and stack into an (N, 3) point cloud array
    points_3d = np.column_stack((
        X_opt[valid_mask], 
        Y_opt[valid_mask], 
        Z_opt[valid_mask]
    ))

    #  Extract matching colors from the original left image
    # CARLA images default to BGRA (4 channels), so slice off the alpha channel
    colors_3d = left_img_array[valid_mask][:, :3]

    return points_3d,colors_3d




