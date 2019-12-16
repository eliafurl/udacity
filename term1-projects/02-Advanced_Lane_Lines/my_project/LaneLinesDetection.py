import lane_lines_detection_tools as tools
import cv2
import numpy as np

class LaneLinesDetection:
    '''
    DONE    Compute the camera calibration matrix and distortion coefficients given a set of chessboard images.

    DONE    Apply a distortion correction to raw images.

    DONE    Use color transforms, gradients, etc., to create a thresholded binary image.

            Apply a perspective transform to rectify binary image (\"birds-eye view\").

            Detect lane pixels and fit to find the lane boundary.

            Determine the curvature of the lane and vehicle position with respect to center.

            Warp the detected lane boundaries back onto the original image.

            Output visual display of the lane boundaries and numerical estimation of lane curvature and vehicle position
    '''
    def __init__(self, cal_imgs_path, display):
        self.calibration_done = 0
        self.cal_imgs_path = cal_imgs_path
        self.calibration = None
        self.display = display
        self.current_frame = None

    # lane lines detection pipeline
    def process_image(self, image):

        self.current_frame = image # DEBUG

        # Calibrate the camera if it is not calibrated
        if not self.calibration_done:
            self.calibration = tools.calibrate_camera(self.cal_imgs_path)
        self.calibration_done = 1

        # Undistort the image
        undistort_image = tools.undistort_image(image, self.calibration[0], self.calibration[1])

        if self.display:
            cv2.imshow('undistort_image', undistort_image)
            cv2.waitKey(0)
            cv2.imwrite('../output_images/undistort_image.jpg',undistort_image)
            cv2.destroyAllWindows()

        # Convert to a binary image and apply a region of interest
        binary_image = tools.convert_to_binary(undistort_image, thresh=(90,255))

        imshape = binary_image.shape
        vertices = np.array([[(0.1*imshape[1],imshape[0]),(0.45*imshape[1], 0.6*imshape[0]), (0.55*imshape[1], 0.6*imshape[0]), (0.9*imshape[1],imshape[0])]], dtype=np.int32)

        masked_binary_image = tools.region_of_interest(binary_image, vertices)

        if self.display:
            cv2.imshow('masked_binary_image', masked_binary_image*254)
            cv2.waitKey(0)
            cv2.imwrite('../output_images/masked_binary_image.jpg', masked_binary_image*254)
            cv2.destroyAllWindows()

        # Apply a perspective transformation




