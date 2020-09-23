# SFND 2D Feature Tracking

<img src="images/keypoints.png" width="820" height="248" />

The idea of the camera course is to build a collision detection system - that's the overall goal for the Final Project. As a preparation for this, you will now build the feature tracking part and test various detector / descriptor combinations to see which ones perform best. This mid-term project consists of four parts:

* First, you will focus on loading images, setting up data structures and putting everything into a ring buffer to optimize memory load. 
* Then, you will integrate several keypoint detectors such as HARRIS, FAST, BRISK and SIFT and compare them with regard to number of keypoints and speed. 
* In the next part, you will then focus on descriptor extraction and matching using brute force and also the FLANN approach we discussed in the previous lesson. 
* In the last part, once the code framework is complete, you will test the various algorithms in different combinations and compare them with regard to some performance measures. 

See the classroom instruction and code comments for more details on each of these parts. Once you are finished with this project, the keypoint matching part will be set up and you can proceed to the next lesson, where the focus is on integrating Lidar points and on object detection using deep-learning. 

## Dependencies for Running Locally
* cmake >= 2.8
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1 (Linux, Mac), 3.81 (Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* OpenCV >= 4.1
  * This must be compiled from source using the `-D OPENCV_ENABLE_NONFREE=ON` cmake flag for testing the SIFT and SURF detectors.
  * The OpenCV 4.1.0 source code can be found [here](https://github.com/opencv/opencv/tree/4.1.0)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools](https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory in the top level directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./2D_feature_tracking`.

## Benchmarking 
I have created bash script generatecom.sh which loops through all the possible combination of detector and descriptor pairs.

Run it: ./generatecom.sh.

This creates a output/output.md file.

I used KNN match selection (k=2) and performed descriptor distance ratio filtering with t=0.8 in file matching2D.cpp.


|Sr. No. | Detector + Descriptor |Total Keypoints |Total Matches |Total Time (ms) |Ratio (matches/time) |
|:---:|:---:|:----:|:-----:|:-----:|:-----:|
|1 | SHITOMASI + BRISK |13423 |2255 |489.655 |4.60529 |
|2 | SHITOMASI + BRIEF |13423 |3234 |482.639 |6.70066 |
|3 | SHITOMASI + ORB |13423 |2856 |691.27 |4.13152 |
|4 | SHITOMASI + FREAK |13423 |2299 |796.32 |2.88703 |
|5 | SHITOMASI + SIFT |13423 |3213 |635.074 |5.05926 |
|6 | HARRIS + BRISK |728 |219 |356.426 |0.614433 |
|7 | HARRIS + BRIEF |728 |257 |357.128 |0.719629 |
|8 | HARRIS + ORB |728 |252 |616.469 |0.40878 |
|9 | HARRIS + FREAK |728 |209 |679.892 |0.307402 |
|10 | HARRIS + SIFT |728 |256 |500.891 |0.511089 |
|11 | FAST + BRISK |17874 |3170 |206.208 |15.3728 |
|12 | FAST + BRIEF |17874 |4904 |200.151 |24.5016 |
|13 | FAST + ORB |17874 |4254 |394.814 |10.7747 |
|14 | FAST + FREAK |17874 |3164 |525.243 |6.02388 |
|15 | FAST + SIFT |17874 |5734 |608.286 |9.42648 |
|16 | BRISK + BRISK |27116 |5073 |3552.64 |1.42795 |
|17 | BRISK + BRIEF |27116 |7474 |3633.46 |2.05699 |
|18 | BRISK + ORB |27116 |5095 |4369.4 |1.16606 |
|19 | BRISK + FREAK |27116 |5008 |3860.58 |1.29721 |
|20 | BRISK + SIFT |27116 |6934 |4716.69 |1.4701 |
|21 | ORB + BRISK |5000 |1378 |623.865 |2.20881 |
|22 | ORB + BRIEF |5000 |1403 |603.852 |2.32342 |
|23 | ORB + ORB |5000 |1466 |1569.9 |0.933817 |
|24 | ORB + FREAK |5000 |627 |897.253 |0.6988 |
|25 | ORB + SIFT |5000 |1578 |1387.89 |1.13698 |
|26 | AKAZE + BRISK |13429 |3240 |1823.38 |1.77692 |
|27 | AKAZE + BRIEF |13429 |4041 |1829.08 |2.2093 |
|28 | AKAZE + ORB |13429 |3340 |2422.53 |1.37873 |
|29 | AKAZE + FREAK |13429 |3228 |2112.13 |1.52832 |
|30 | AKAZE + AKAZE |13429 |3463 |3295.21 |1.05092 |
|31 | AKAZE + SIFT |13429 |3633 |2135.31 |1.70139 |
|32 | SIFT + BRISK |13860 |2458 |874.397 |2.81108 |
|33 | SIFT + BRIEF |13860 |3243 |914.507 |3.54617 |
|34 | SIFT + FREAK |13860 |2428 |1241.34 |1.95595 |
|35 | SIFT + SIFT |13860 |2557 |1497.76 |1.70722 |

Total combinations were 42, but for some of the combinations got an OpenCV error:
OpenCV(4.1.0) Error: Assertion failed


### Top 3 detector/ descriptor pairs with maximum Ratio
|Sr. No. | Detector + Descriptor |Total Keypoints |Total Matches |Total Time (ms) |Ratio (matches/time) |
|:---:|:---:|:----:|:-----:|:-----:|:-----:|
|12 | FAST + BRIEF |17874 |4904 |200.151 |24.5016 |
|11 | FAST + BRISK |17874 |3170 |206.208 |15.3728 |
|13 | FAST + ORB |17874 |4254 |394.814 |10.7747 |
