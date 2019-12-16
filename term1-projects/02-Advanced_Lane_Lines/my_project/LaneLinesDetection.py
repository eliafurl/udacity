import lane_lines_detection_tools as tools
from Line import Line
import cv2
import numpy as np

class LaneLinesDetection:
    '''
    DONE    Compute the camera calibration matrix and distortion coefficients given a set of chessboard images.

    DONE    Apply a distortion correction to raw images.

    DONE    Use color transforms, gradients, etc., to create a thresholded binary image.

    DONE    Apply a perspective transform to rectify binary image (\"birds-eye view\").

        Detect lane pixels and fit to find the lane boundary.

            Determine the curvature of the lane and vehicle position with respect to center.

            Warp the detected lane boundaries back onto the original image.

            Output visual display of the lane boundaries and numerical estimation of lane curvature and vehicle position
    '''
    def __init__(self, cal_imgs_path, display):
        self.calibration_done = 0
        self.cal_imgs_path = cal_imgs_path
        self.calibration = np.array([])
        self.display = display
        #self.current_frame = None
        self.M = np.array([])
        self.left_lane = None
        self.right_lane = None

    # lane lines detection pipeline
    def process_image(self, image):

        #self.current_frame = image # DEBUG

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
        binary_image = tools.convert_to_binary(undistort_image, thresh=(70,255))

        imshape = binary_image.shape
        vertices = np.array([[(0.1*imshape[1],imshape[0]),(0.45*imshape[1], 0.6*imshape[0]), (0.55*imshape[1], 0.6*imshape[0]), (0.9*imshape[1],imshape[0])]], dtype=np.int32)

        masked_binary_image = tools.region_of_interest(binary_image, vertices)

        if self.display:
            cv2.imshow('masked_binary_image', masked_binary_image*254)
            cv2.waitKey(0)
            cv2.imwrite('../output_images/masked_binary_image.jpg', masked_binary_image*254)
            cv2.destroyAllWindows()

        # Compute the perspective transformation matrix M
        if not self.M.any():
            src_points = np.float32([[200,720], [615,435], [665,435], [1080,720]])
            offset = 300
            dst_points = np.float32([[offset,imshape[0]], [offset,0], [imshape[1]-offset,0], [imshape[1]-offset,imshape[0]]])

            if self.display:
                for_displaying = masked_binary_image.copy()
                cv2.polylines(for_displaying,np.int32([src_points]),True,(255,0,0))
                cv2.imshow('Perspective transform before', for_displaying)
                cv2.waitKey(0)
                cv2.imwrite('../output_images/before_transformation.jpg',for_displaying)
                cv2.destroyAllWindows()

            self.M = tools.getPerspectiveTransform(src_points, dst_points)

        # Apply a perspective transform to rectify binary image.
        top_down_view_image = tools.top_down_view(masked_binary_image, self.M)

        if self.display:
            for_displaying = top_down_view_image.copy()
            cv2.polylines(for_displaying,np.int32([dst_points]),True,(255,255,0))
            cv2.imshow('top_down_view_image', for_displaying)
            cv2.waitKey(0)
            cv2.imwrite('../output_images/top_down_view_image.jpg', for_displaying)
            cv2.destroyAllWindows()

        # Detect lane pixels and fit to find the lane boundary.
        self.left_line, self.right_line = tools.find_lanes_boundaries(img)


