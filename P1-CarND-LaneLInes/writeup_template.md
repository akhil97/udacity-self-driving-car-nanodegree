# **Finding Lane Lines on the Road** 

## Writeup Template

### You can use this file as a template for your writeup if you want to submit it as a markdown file. But feel free to use some other method and submit a pdf if you prefer.

---

**Finding Lane Lines on the Road**

The goals / steps of this project are the following:
* Make a pipeline that finds lane lines on the road
* Reflect on your work in a written report


[//]: # (Image References)

[image1]: ./examples/grayscale.jpg "Grayscale"

---

### Reflection

### 1. Describe your pipeline. As part of the description, explain how you modified the draw_lines() function.

My pipeline consisted of 5 steps. First, I converted the images to grayscale, then I applied Gaussian blurr on the grayscale image. Then I applied Canny Edge Detector using appropriate values for threshold. Then I used the region of interest function to highlight the desired region by using the image obtained after applying Canny Edge Detector. Then I applied Hough transform and used the weighted image funtion to overlay the original image and the one obtained from Hough transform.  


### 2. Identify potential shortcomings with your current pipeline


One potential shortcoming would be what would happen when the video plays there is some amount of distortion. 

Another shortcoming could be that there is some amount of deviation of the lines from the lanes as the video is being played.


### 3. Suggest possible improvements to your pipeline

A possible improvement would be to reduce the amount of distortion when the video is being played.

Another potential improvement could be to adjust parameters to reduce deviation of lines from the lanes accordingly.
