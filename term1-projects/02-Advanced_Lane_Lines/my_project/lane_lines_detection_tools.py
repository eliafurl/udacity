#imports
import numpy as np
import cv2

#useful methods
def undistort_image(img, cameraMatrix, distCoeffs):
    undist = cv2.undistort(img, cameraMatrix, distCoeffs, None, cameraMatrix)
    return undist

def hls_threshold(img, h_thresh=(0, 255), s_thresh=(0, 255)):
    hls = cv2.cvtColor(img, cv2.COLOR_BGR2HLS)
    # H channel
    h_channel = hls[:,:,0]
    h_threshold = np.zeros_like(img)
    h_threshold[(h_channel > h_thresh[0]) & (h_channel <= h_thresh[1])] = 1
    cv2.imshow('h_threshold', h_threshold*254) #DEBUG
    cv2.waitKey(0) # DEBUG
    # cv2.imwrite('./output_images/h_threshold.jpg',h_threshold*254)
    # S channel
    s_channel = hls[:,:,2]
    s_threshold = np.zeros_like(img)
    s_threshold[(s_channel > s_thresh[0]) & (s_channel <= s_thresh[1])] = 1
    cv2.imshow('s_threshold', s_threshold*254) #DEBUG
    cv2.waitKey(0) # DEBUG
    # cv2.imwrite('./output_images/s_threshold.jpg',s_threshold*254)
    hls_threshold = np.zeros_like(img)
    hls_threshold[(h_threshold == 1) & (s_threshold == 1)] = 1
    cv2.imshow('hls_threshold', hls_threshold*254) #DEBUG
    cv2.waitKey(0) # DEBUG
    # cv2.imwrite('./output_images/hls_threshold.jpg',hls_threshold*254)
    return hls_threshold

def red_threshold(img, thresh=(0,255)):
    red_channel = img[:,:,2]
    red_threshold = np.zeros_like(img)
    red_threshold[(red_channel > thresh[0]) & (red_channel <= thresh[1])] = 1
    cv2.imshow('red_threshold', red_threshold*254) #DEBUG
    cv2.waitKey(0) # oldDEBUG
    # cv2.imwrite('./output_images/r_threshold.jpg',r_threshold*254)
    return red_threshold

def convert_to_binary(img):
    red_binary = red_threshold(img, thresh=(150,255))
    hls_binary = hls_threshold(img, h_thresh=(15,100), s_thresh=(90,255))
    binary_output = np.zeros_like(img)
    binary_output[(red_binary == 1) & (hls_binary == 1)] = 1
    return binary_output

def region_of_interest(img, vertices):
    #defining a blank mask to start with
    mask = np.zeros_like(img)

    #defining a 3 channel or 1 channel color to fill the mask with depending on the input image
    if len(img.shape) > 2:
        channel_count = img.shape[2]  # i.e. 3 or 4 depending on your image
        ignore_mask_color = (255,) * channel_count
    else:
        ignore_mask_color = 255

    #filling pixels inside the polygon defined by "vertices" with the fill color
    cv2.fillPoly(mask, vertices, ignore_mask_color)

    #returning the image only where mask pixels are nonzero
    masked_image = cv2.bitwise_and(img, mask)
    return masked_image

def getPerspectiveTransform(src_points, dst_points):
    M = cv2.getPerspectiveTransform(src_points, dst_points)
    return M

def top_down_view(img, M):
    img_size = (img.shape[1],img.shape[0])
    warped = cv2.warpPerspective(img, M, img_size, flags=cv2.INTER_LINEAR)
    return warped

def hist(img):
    bottom_half = img[img.shape[0]//2:,:]
    histogram = np.sum(bottom_half, axis=0)
    return histogram

def find_lanes_boundaries(img):
    pass

def find_lines_sliding_window(img):
    pass