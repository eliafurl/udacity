# **Finding Lane Lines on the Road** 

[//]: # (Image References)

[image1]: ./imgs/grayscale_image.jpg "Grayscale"
[image2]: ./imgs/filtered_image.jpg "Filtered image"
[image3]: ./imgs/canny_image.jpg "Canny edge detection"
[image4]: ./imgs/masked_image.jpg "Masked image for line detection"
[image5]: ./imgs/lines_image.jpg "Line image"
[image6]: ./imgs/final_image.jpg "Final processed image"

---

### Reflection

### 1. Describe your pipeline. As part of the description, explain how you modified the draw_lines() function.

My pipeline consists of 7 steps:
1. converts the image to grayscale 

    ![alt_text][image1]

1. smooths the image applying a gaussian filtering in order to decrease the noise
    
    ![alt_text][image2]
    
1. applies the Canny edge detection to the converted and filtered image
    
    ![alt_text][image3]
    
1. applies the ROI mask to the Canny image in order to search the lines only in the interested region
    
    ![alt_text][image4]
    
1. search for the lines with the Hough transformation
1. compute the average lane lines from the detected lines:
    1. the average slope and lines segments are evaluated
    1. the average line segment is extended in order to be displayed in the whole ROI
    1. the final lines are drawn on a black image
    
    ![alt text][image5]
    
1. the original image and the processed one are merged in order to superimpose the obtained lines to the road
    
    ![alt text][image6]

### 2. Identify potential shortcomings with your current pipeline

This pipeline relies only on the different colours of the pixels. In this way the different environmental conditions (e.g. bright sun, night, rain) could very likely affect the performance of this method.   


### 3. Suggest possible improvements to your pipeline

In order to overcome the limitation highlighted before, the pipeline could be tuned for the different environmental conditions. Then before starting the processing the environmental condition should be identified (e.g maybe looking at the histogram of the picture) and the parameters for the specific condition should be used.

This pipeline can be furthermore improved in two possible ways:
* for detecting better the yellow lines we could filter separately the yellow and white pixels creating two different masks and then combining them together for creating a high contrast image for the edge detection
* implementing dynamic ROI, exploiting the horizon line
