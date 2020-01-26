import lane_lines_detection_tools as tools
from Line import Line
import cv2
import numpy as np
import glob

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
    def __init__(self, display):
        self.calibration_done = 0
        self.cal_imgs_path = ''
        self.cameraMatrix = np.array([])
        self.distCoeffs = np.array([])
        self.display = display
        self.frame_number = 0
        self.window_search = True
        self.M = np.array([])
        self.M_inv = np.array([])
        self.left_line = Line()
        self.right_line = Line()
        # self.video_mode = 0

    def calibrate_camera(self, path):
        '''
        Calibration of the camera
        inputs:     path = path of the calibration images folder
        outputs:    (mtx, dist)
        '''
        self.cal_imgs_path = path
        # prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
        objp = np.zeros((6*9,3), np.float32)
        objp[:,:2] = np.mgrid[0:9,0:6].T.reshape(-1,2)

        # Arrays to store object points and image points from all the images.
        objpoints = [] # 3d points in real world space
        imgpoints = [] # 2d points in image plane.

        # Make a list of calibration images
        images = glob.glob(path)

        # Step through the list and search for chessboard corners
        for fname in images:
            img = cv2.imread(fname)
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

            # Find the chessboard corners
            ret, corners = cv2.findChessboardCorners(gray, (9,6), None)

            # If found, add object points, image points
            if ret == True:
                objpoints.append(objp)
                imgpoints.append(corners)

                # Draw and display the corners
                #img = cv2.drawChessboardCorners(img, (9,6), corners, ret)
                #cv2.imshow('img',img)
                #cv2.waitKey(50)

        #cv2.destroyAllWindows()

        ret, cameraMatrix, distCoeffs, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)

        self.cameraMatrix = cameraMatrix
        self.distCoeffs = distCoeffs
        self.calibration_done = 1

    # lane lines detection pipeline
    def process_image(self, image):

        #self.current_frame = image # DEBUG

        # Warning if the camera is not calibrated
        if not self.calibration_done:
            print('Warning: the camera is not calibrated!')

        # Undistort the image
        undistort_image = tools.undistort_image(image, self.cameraMatrix, self.distCoeffs)

        if self.display:
            cv2.imshow('undistort_image', undistort_image)
            cv2.waitKey(0)
            cv2.imwrite('./output_images/01-undistort_image.jpg',undistort_image)
            cv2.destroyAllWindows()

        # Convert to a binary image and apply a region of interest
        binary_image = tools.convert_to_binary(undistort_image, self.display)

        imshape = binary_image.shape
        vertices = np.array([[(0.1*imshape[1],imshape[0]),(0.45*imshape[1], 0.6*imshape[0]), (0.55*imshape[1], 0.6*imshape[0]), (0.9*imshape[1],imshape[0])]], dtype=np.int32)

        masked_binary_image = tools.region_of_interest(binary_image, vertices)

        if self.display:
            out_img = np.dstack((masked_binary_image, masked_binary_image, masked_binary_image))*255
            cv2.imshow('masked_binary_image', out_img)
            cv2.waitKey(10)
            cv2.imwrite('./output_images/07-masked_binary_image.jpg', out_img)
            cv2.destroyAllWindows()

        # Compute the perspective transformation matrix M
        if not self.M.any():
            x = imshape[1]
            y = imshape[0]
            source_points = np.float32([[0.117 * x, y],[(0.5 * x) - (x*0.078), (2/3)*y],[(0.5 * x) + (x*0.078), (2/3)*y],[x - (0.117 * x), y]])
            destination_points = np.float32([[0.25 * x, y],[0.25 * x, 0],[x - (0.25 * x), 0],[x - (0.25 * x), y]])
            
            if self.display:
                out_img = image.copy()
                cv2.polylines(out_img,np.int32([source_points]),True,(0,0,255), 5)
                cv2.imshow('Perspective transform before', out_img)
                cv2.waitKey(10)
                cv2.imwrite('./output_images/08-before_transformation.jpg',out_img)
                cv2.destroyAllWindows()

            self.M = tools.getPerspectiveTransform(source_points, destination_points)
            self.M_inv = tools.getPerspectiveTransform(destination_points, source_points)
        # Apply a perspective transform to rectify binary image.
        top_down_view_image = tools.top_down_view(masked_binary_image, self.M) #masked_binary_image

        if self.display:
            out_img = np.dstack((top_down_view_image, top_down_view_image, top_down_view_image))*255
            print('out_img.shape = {}'.format(out_img.shape))
            cv2.polylines(out_img,np.int32([destination_points]),True,(0,0,255), 5)
            cv2.imshow('Perspective transform after', out_img)
            cv2.waitKey(10)
            cv2.imwrite('./output_images/09-top_down_view_image.jpg', out_img)
            cv2.destroyAllWindows()

        # Detect lane pixels and fit to find the lane boundary.
        if self.window_search:
            left_fit, right_fit, leftx, lefty, rightx, righty, = tools.find_lines_sliding_window(top_down_view_image, self.display)
        else:
            left_fit, right_fit, leftx, lefty, rightx, righty, self.window_search = tools.find_lines_from_prior(top_down_view_image, self.left_line.current_fit, self.right_line.current_fit, self.window_search, self.frame_number, self.display)

        self.left_line.update(left_fit, leftx, lefty)
        self.right_line.update(right_fit, rightx, righty)

        # highlight lane boundaries
        highlighted_lane = tools.lane_fill_poly(top_down_view_image, undistort_image, self.left_line.current_fit, self.right_line.current_fit, self.M_inv)

        if self.display:
            #cv2.imshow('highlighted_lane', highlighted_lane)
            #cv2.waitKey(0)
            cv2.imwrite('./output_images/11-highlighted_lane.jpg',highlighted_lane)
            cv2.destroyAllWindows()

        if self.frame_number==0 or self.frame_number%15==0:
            # measure curve radius
            curve_radius = tools.measure_curve(top_down_view_image, self.left_line.current_fit, self.right_line.current_fit)
            print('curve_radius = {}'.format(curve_radius))
            # measure vehicle offset from the center of the lane
            vehicle_offset = tools.vehicle_offset(highlighted_lane, self.left_line.current_fit, self.right_line.current_fit)
            print('vehicle_offset = {}'.format(vehicle_offset))

        font = cv2.FONT_HERSHEY_TRIPLEX
        processed_frame = cv2.putText(highlighted_lane, 'Radius: '+str(curve_radius)+' m', (30, 40), font, 1, (0,255,0), 2)
        processed_frame = cv2.putText(processed_frame, 'Offset: '+str(vehicle_offset)+' m', (30, 80), font, 1, (0,255,0), 2)
        
        if self.display:
            #cv2.imshow('processed_frame', processed_frame)
            #cv2.waitKey(0)
            cv2.imwrite('./output_images/12-processed_frame.jpg',processed_frame)
            cv2.destroyAllWindows()
        
        self.frame_number += 1

