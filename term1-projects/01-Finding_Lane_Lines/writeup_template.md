# **Finding Lane Lines on the Road** 

[//]: # (Image References)

[image0]: ./examples/grayscale.jpg "Grayscale"
[image1]: ./imgs/grayscale_image.jpg "Grayscale"
[image2]: ./imgs/canny_image.jpg "Canny edge detection"
[image3]: ./imgs/masked_image.jpg "Masked image for line detection"

---

### Reflection

### 1. Describe your pipeline. As part of the description, explain how you modified the draw_lines() function.

My pipeline consisted of <N> steps:
1. converts the image to grayscale

![alt_text][image1]
1. smooths the image applying a gaussian filtering in order to decrease the noise
1. applies the Canny edge detection to the converted and filtered image

![alt_text][image2]
1. applies the ROI mask to the Canny image in order to search the lines only in the interested region

![alt_text][image3]
1. search for the lines with the Hough transformation
1. compute the average lane lines from the detected lines:
    1. the average slope and lines segments are evaluated
    1. the line segment is augmented in order to be displayed in the whole ROI
    1. 
    
In order to draw a single line on the left and right lanes, I modified the draw_lines() function by ...

If you'd like to include images to show how the pipeline works, here is how to include an image: 

![alt text][image0]


### 2. Identify potential shortcomings with your current pipeline

This pipeline relies only on the different colours of the pixels. In this way the different environmental conditions (e.g. bright sun, night, rain) could very likely affect the performance of this method.   


### 3. Suggest possible improvements to your pipeline

In order to overcome the limitation highlighted before, the pipeline could be tuned for the different environmental conditions. Then before starting the processing the environmental condition should be identified (e.g maybe looking at the histogram of the picture) and the parameters for the specific condition should be used.

This pipeline can be furthermore improved in two possible ways:
* for detecting better the yellow lines we could filter separately the yellow and white pixels creating two different masks and then combining them together for creating a high contrast image for the edge detection
* implementing dynamic ROI, exploiting the horizon line
