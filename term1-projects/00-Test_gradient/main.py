import numpy as np
import cv2
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import pickle


def abs_sobel_thresh(img, orient='x', sobel_kernel=3, thresh=(0, 255)):
    # Apply the following steps to img
    # 1) Convert to grayscale
    gray_img = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)

    # 2) Take the derivative in x or y given orient = 'x' or 'y'
    sobel = np.zeros(img.shape)
    if orient == 'x':
        sobel = cv2.Sobel(gray_img, cv2.CV_64F, 1, 0)
    elif orient == 'y':
        sobel = cv2.Sobel(gray_img, cv2.CV_64F, 0, 1)
    else:
        print("Not a known gradient orientation.")
        return 1

    # 3) Take the absolute value of the derivative or gradient
    abs_sobel = np.absolute(sobel)

    # 4) Scale to 8-bit (0 - 255) then convert to type = np.uint8
    scaled_sobel = np.uint8(255*abs_sobel/np.max(abs_sobel))

    # 5) Create a mask of 1's where the scaled gradient magnitude
    sobel_mask = [(scaled_sobel >= thresh[0]) & (scaled_sobel <= thresh[1])]

    # 6) Return this mask as your grad_binary image
    grad_binary = np.zeros_like(scaled_sobel)
    grad_binary[sobel_mask] = 1
    return grad_binary

def mag_thresh(image, sobel_kernel=3, mag_thresh=(0, 255)):
    # Apply the following steps to img
    # 1) Convert to grayscale
    gray_img = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)

    # 2) Take the gradient in x and y separately
    gradient_x = cv2.Sobel(gray_img, cv2.CV_64F, 1, 0, ksize=sobel_kernel)
    gradient_y = cv2.Sobel(gray_img, cv2.CV_64F, 0, 1, ksize=sobel_kernel)

    # 3) Calculate the magnitude
    magnitude = np.sqrt(np.square(gradient_x) + np.square(gradient_y))

    # 4) Scale to 8-bit (0 - 255) and convert to type = np.uint8
    scaled_magnitude = np.uint8(255*magnitude/np.max(magnitude))

    # 5) Create a binary mask where mag thresholds are met
    magnitude_mask = [(scaled_magnitude >= mag_thresh[0]) & (scaled_magnitude <= mag_thresh[1])]

    # 6) Return this mask as your mag_binary image
    mag_binary = np.zeros_like(gray_img)
    mag_binary[magnitude_mask] = 1
    return mag_binary

def dir_threshold(image, sobel_kernel=3, thresh=(0, np.pi/2)):
    # Apply the following steps to img
    # 1) Convert to grayscale
    gray_img = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)

    # 2) Take the gradient in x and y separately
    gradient_x = cv2.Sobel(gray_img, cv2.CV_64F, 1, 0, ksize=sobel_kernel)
    gradient_y = cv2.Sobel(gray_img, cv2.CV_64F, 0, 1, ksize=sobel_kernel)

    # 3) Take the absolute value of the x and y gradients
    abs_gradient_x = np.abs(gradient_x)
    abs_gradient_y = np.abs(gradient_y)

    # 4) Use np.arctan2(abs_sobely, abs_sobelx) to calculate the direction of the gradient
    gradient_direction = np.arctan2(abs_gradient_y, abs_gradient_x)

    # 5) Create a binary mask where direction thresholds are met
    gradient_direction_mask = [(gradient_direction >= thresh[0]) & (gradient_direction <= thresh[1])]
    # 6) Return this mask as your dir_binary image
    dir_binary = np.zeros_like(gray_img)
    dir_binary[gradient_direction_mask] = 1

    return dir_binary

if __name__ == '__main__':
    print('WIP')
    image = mpimg.imread('signs_vehicles_xygrad.png')

    # Choose a Sobel kernel size
    ksize = 15 # Choose a larger odd number to smooth gradient measurements

    # Apply each of the thresholding functions
    gradx = abs_sobel_thresh(image, orient='x', sobel_kernel=ksize, thresh=(40, 100))
    grady = abs_sobel_thresh(image, orient='y', sobel_kernel=ksize, thresh=(50, 100))
    mag_binary = mag_thresh(image, sobel_kernel=ksize, mag_thresh=(30, 100))
    dir_binary = dir_threshold(image, sobel_kernel=ksize, thresh=(0.7, 1.3))

    combined = np.zeros_like(dir_binary)
    combined[((gradx == 1) & (grady == 1)) | ((mag_binary == 1) & (dir_binary == 1))] = 1

    # Plot the result
    # f, (ax1, ax2, ax3, ax4, ax5) = plt.subplots(1, 5, figsize=(24, 9))
    # f.tight_layout()
    # ax1.imshow(image)
    # ax1.set_title('Original Image', fontsize=20)
    # ax2.imshow(gradx, cmap='gray')
    # ax2.set_title('Thresholded gradx', fontsize=20)
    # ax3.imshow(grady, cmap='gray')
    # ax3.set_title('Thresholded grady', fontsize=20)
    # ax4.imshow(mag_binary, cmap='gray')
    # ax4.set_title('Thresholded mag_binary', fontsize=20)
    # ax5.imshow(dir_binary, cmap='gray')
    # ax5.set_title('Thresholded dir_binary', fontsize=20)
    # plt.subplots_adjust(left=0., right=1, top=0.9, bottom=0.)
    # plt.show()

    plt.imshow(combined, cmap='gray')
    plt.show()
    # cv2.imshow('test - dir_binary',image)
    # cv2.waitKey(0)
    # cv2.destroyAllWindows()