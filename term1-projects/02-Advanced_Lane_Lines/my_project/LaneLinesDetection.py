import lane_lines_detection_tools as tools
import cv2

class LaneLinesDetection:
    '''
    DONE    Compute the camera calibration matrix and distortion coefficients given a set of chessboard images.
    
    DONE    Apply a distortion correction to raw images.
    
            Use color transforms, gradients, etc., to create a thresholded binary image.
    
            Apply a perspective transform to rectify binary image (\"birds-eye view\").
    
            Detect lane pixels and fit to find the lane boundary.
    
            Determine the curvature of the lane and vehicle position with respect to center.
    
            Warp the detected lane boundaries back onto the original image.
    
            Output visual display of the lane boundaries and numerical estimation of lane curvature and vehicle position
    '''
    def __init__(self, cal_imgs_path, display):
        self.calibration_done = 0
        self.cal_imgs_path = cal_imgs_path
        self.calibration = 0
        self.display = display

    # lane lines detection pipeline
    def process_image(self, image):
        if not self.calibration_done:
            self.calibration = tools.calibrate_camera(self.cal_imgs_path)
        self.calibration_done = 1
    
        undistort_image = tools.undistort_image(image, self.calibration[0], self.calibration[1])
        
        if self.display:
            cv2.imshow('undistort_image', undistort_image)
            cv2.waitKey(0)
            cv2.imwrite('../output_images/undistort_image.jpg',undistort_image)
