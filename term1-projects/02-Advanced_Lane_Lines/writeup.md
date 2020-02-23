## Writeup Advanced Lane Finding Project

The goals / steps of this project are the following:

* Compute the camera calibration matrix and distortion coefficients given a set of chessboard images.
* Apply a distortion correction to raw images.
* Use color transforms, gradients, etc., to create a thresholded binary image.
* Apply a perspective transform to rectify binary image ("birds-eye view").
* Detect lane pixels and fit to find the lane boundary.
* Determine the curvature of the lane and vehicle position with respect to center.
* Warp the detected lane boundaries back onto the original image.
* Output visual display of the lane boundaries and numerical estimation of lane curvature and vehicle position.

[//]: # (Image References)

[image1]: ./output_images/00-original_image.jpg "Original image"
[image2]: ./output_images/01-undistort_image.jpg "Undistorted"
[image3]: ./output_images/03-h_threshold.jpg "Hue Threshold"
[image4]: ./output_images/04-s_threshold.jpg "Saturation Threshold"
[image5]: ./output_images/05-hls_threshold.jpg "HLS Threshold"
[image6]: ./output_images/06-r_threshold.jpg "Red Threshold"
[image7]: ./output_images/06.1-sobel_x_threshold.jpg "Sobel X Threshold"
[image8]: ./output_images/06.1-sobel_y_threshold.jpg "Sobel Y Threshold"
[image9]: ./output_images/07-masked_binary_image.jpg "Binary Image"
[image10]: ./output_images/08-before_transformation.jpg "Before Trasformation"
[image11]: ./output_images/09-top_down_view_image.jpg "Top Down View"
[image12]: ./output_images/10-window_search.jpg "Window Search"
[image13]: ./output_images/11-highlighted_lane.jpg "Highlighted Lane"
[image14]: ./output_images/12-processed_frame.jpg "Processed Frame"
[video1]: ./videos/processed_project_video.mp4 "Video"

## [Rubric](https://review.udacity.com/#!/rubrics/571/view) Points

---

### Camera Calibration

Camera calibration is used to correct the image distortion introduced from the camera sensor.
The calibration procedure has been implemented within the method `calibrate_camera` of the `LaneLinesDetection` class, in the file "LaneLinesDetection.py".


The calibration procedure is based on the analysis of a set of chessboard images. For each image in the given folder, it finds the corners, that are points on the image. Corners are defined as the intersection between two edges. They are detected, after converting the color input image to grayscale, using the OpenCv function `cv2.findChessboardCorners`. At each set of corners image points, a set of object 3D points in world coordinates are associated. The object points are distributed on-grid without distortion. The camera calibration is performed using the function `cv2.calibrateCamera` that allows computing the distortion coefficients and the transformation matrix needed to transform 3D points to 2D points. These computed values are needed to perform distortion correction and are stored in the class `LaneLinesDetection`.

### Pipeline

Now I will explain the whole pipeline implemented in `procces_image` in the class `LaneLinesDetection`. The other useful methods that have been used are implemented instead in the file `lane_lines_detection_tools.py`.
The method `procces_image` is applied to each image, or frame of the video and is composed of the following steps:

#### 1. Distortion correction

First a distrortion correction is performed using the `cameraMatrix` matrix computed before. 
* Original image
![alt text][image1]
* Undistorted image
![alt text][image2]

#### 2. Binary image creation for lines identification

In order to detect lines of different colors under several light condition, I used a combination of color and gradient thresholds to generate a binary image. These are implemented with the function `convert_to_binary` in `lane_lines_detection_tools.py`. 

##### Color thresholding

For the color thresholding these methods have been utilized:
* H channel
![alt text][image3]
* S channel
![alt text][image4]
* Red channel
![alt text][image6]

##### Gradient thresholding

For the gradient thresholding the following methods have been implemented:
* Sobel X derivative
![alt text][image7]
* Sobel Y derivative
![alt text][image8]

##### Final result

After the combination of the thresholding, a mask is applied to retain the information only on the ROI where the lines are located.

![alt text][image9]

#### 3. Perspective transformation

The perspective transformation matrix `M` is computed with the OpenCV function `cv2.getPerspectiveTransform` given the source and destination points defined as follows:

```python
x = imshape[1]
y = imshape[0]
source_points = np.float32([[0.117 * x, y],
    [(0.5 * x) - (x*0.078), (2/3)*y],
    [(0.5 * x) + (x*0.078), (2/3)*y],
    [x - (0.117 * x), y]])
destination_points = np.float32([[0.25 * x, y],
    [0.25 * x, 0],
    [x - (0.25 * x), 0],
    [x - (0.25 * x), y]])
```
This trasformation is applied to the binary image using the function `top_down_view` implemented in `lane_lines_detection_tools.py`.

![alt text][image10]
![alt text][image11]

#### 4. Lane-line detection

The binary image is processed by the function `find_lines_sliding_window` which performs a window search for the line pixels and then fits a second-order polynomial using `np.polyfit`. As a starting position for the window search the histogram of the bottom half of the image is used.

![alt text][image12]

#### 5. Display results

Before visualizing the results, the road curvature and vehicle position with respect to the center of the lane are computed using `measure_curve` and `vehicle_offset` implemented in `lane_lines_detection_tools.py`. The first performs the mean between the right and left lines curvature.

Finally, the detected lane boundaries are highlighted in the original image and the information regarding the curvature and vehicle offset are displayed.

![alt text][image14]

---

### Video result

Here's a [link to my video result](./videos/processed_project_video.mp4)

---

### Discussion
The parser in the `main.py` file allows having a single main file both for the video and the image mode. The program can be run as described in the following, where `mode = {image, video}`:
```bash
python3 my_project/main.py <mode>
```
The modularity of the code allows to modify and improve the pipeline without any major issue. My pipeline could be improved in several ways, such as:
1. performing a sanity check of the polynomial fit before considering it;
1. averaging the results of the polynomial fit of the last `n` iterations for a smoother estimation;
1. using `find_lines_from_prior` instead of the sliding window approach, for increasing the overall performance, if the estimation of the polynomial at the previous step passes the sanity check.
Due to a personal time limitation, I was not able to test these approaches.
