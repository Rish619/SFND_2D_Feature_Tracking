# SFND 2D Feature Tracking

<img src="images/keypoints.png" width="820" height="248" />

The idea of the camera course is to build a collision detection system - that's the overall goal for the Final Project. As a preparation for this, you will now build the feature tracking part and test various detector / descriptor combinations to see which ones perform best. This mid-term project consists of four parts:

* First, you will focus on loading images, setting up data structures and putting everything into a ring buffer to optimize memory load. 
* Then, you will integrate several keypoint detectors such as HARRIS, FAST, BRISK and SIFT and compare them with regard to number of keypoints and speed. 
* In the next part, you will then focus on descriptor extraction and matching using brute force and also the FLANN approach we discussed in the previous lesson. 
* In the last part, once the code framework is complete, you will test the various algorithms in different combinations and compare them with regard to some performance measures. 

See the classroom instruction and code comments for more details on each of these parts. Once you are finished with this project, the keypoint matching part will be set up and you can proceed to the next lesson, where the focus is on integrating Lidar points and on object detection using deep-learning. 

# Keypoint Detector,Descriptor and Matching in action

<img src="images/Result2.gif" width="820" height="248" />

Two consecutive frames compared on the basis of the keypoints detected(SHITOMASI AND BRISK Detector/Descriptor) K nearest neighbour matching(k=2) with Descriptor Distance Ratio as 0.8(threshold).

* TTC(Time to collison) or distance travelled by the car can also be calculated by analysing the moving Mono camera frame stream and matching the key features and change in their size with respect to the entire frame(relative comparison).

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
5. Currently to try detector,descriptor and match combination, one must manually substitute the values 
in MidTermProject_Camera_Student.cpp and then run `make` in the build directory and 
then run `./2D_feature_tracking` and record the values. 


## Marking Guide/Rubric for this Assignment

1. Data Buffer Optimization: Implement a vector for dataBuffer objects whose size does not exceed a limit (e.g. 2 elements). 

   

   > This can be achieved by pushing in new elements on one end and removing elements on the other end. First of all, we push the current image into data frame buffer, then we check if the buffer size is more then the max buffer size(dataBufferSize), if so we will pop out the first element to ensure the data frame buffer will not accumulate.

   

2. Keypoint Detection: Implement detectors Shi-Thomasi, HARRIS, FAST, BRISK, ORB, AKAZE, and SIFT and make them selectable by setting a string accordingly.

   

   >  Based on the input string, we implemented 3 different functions: detKeypointsShiTomasi (Shi-Thomasi, Good Features to Track), detKeypointsHarris(Harris corner), detKeypointsModern(FAST, BRISK, ORB, AKAZE, and SIFT). In each function, we set up several parameters(block size, minimal distance, threshold...). Then OpenCV build-in detector class will be initialized with these parameters and scan the whole image to detect key-points. Also the detection time will be logged for performance evaluation.

   

3. Keypoint Removal: Remove all keypoints outside of a pre-defined rectangle and only use the keypoints within the rectangle for further processing.

   

   > For this project, we will focus more on the preceding vehicle, so we set up a certain region of interest around the middle of the whole image. We will only keep the key-point that is in this area to reduce noise and save computation power for following steps.

   

4. Keypoint Descriptors: Implement descriptors BRIEF, ORB, FREAK, AKAZE and SIFT and make them selectable by setting a string accordingly.

   

   > Descriptor is a vector of values, which describes the image patch around a keypoint. Similar to step 2, we implement a function to detection descriptors based on the input string. We still use OpenCV build-in descriptors (BRIEF, ORB, FREAK, AKAZE and SIFT) class with default parameters to uniquely identify keypoints. Similar to step 2, we log the descriptor extraction time for performance evaluation.

   

5. Descriptor Matching: Implement FLANN matching as well as k-nearest neighbor selection. Both methods must be selectable using the respective strings in the main function.

   

   >User can choose which matching method to use: Brute-force matcher(MAT_BF) or FLANN matcher(MAT_FLANN). Note that we need to convert our image to CV_32F type due to  a bug in current OpenCV implementation if we choose FLANN matching.

   

6. Descriptor Distance Ratio: Use the K-Nearest-Neighbor matching to implement the descriptor distance ratio test, which looks at the ratio of best vs. second-best match to decide whether to keep an associated pair of keypoints.

   

   >Both nearest neighbor (best match) and K nearest neighbors (default k=2) are implemented. For KNN matching, we filter matches using descriptor distance ratio test to remove some outliers. In current implementation, only following match will be accepted:
   >
   > knn_match[0].distance < 0.9 * knn_match[1].distance

   

7. Performance Evaluation 1: Count the number of keypoints on the preceding vehicle for all 10 images and take note of the distribution of their neighborhood size. Do this for all the detectors you have implemented.

   

   [Performance Evaluation 1 - Number of Key-points for 10 Images](#benchmark)

   

8. Performance Evaluation 2: Count the number of matched keypoints for all 10 images using all possible combinations of detectors and descriptors. In the matching step, the BF approach is used with the descriptor distance ratio set to 0.8.

   

   [Performance Evaluation 2 - Number of Matched Key-points for 10 Images](#benchmark)

   

9. Performance Evaluation 3: Log the time it takes for keypoint detection and descriptor extraction. The results must be entered into a spreadsheet and based on this data, the TOP3 detector / descriptor combinations must be recommended as the best choice for our purpose of detecting keypoints on vehicles.

   

   [Performance Evaluation 3 - Key-point Detection and Descriptor Extraction Time Consumption](#benchmark)



## Benchmarking 

#### Number of Key-points for 10 Images

| Detectors | Number of Key-points |
| :-------: | :------------------: |
| SHITOMASI |        13423         |
|  HARRIS   |         1737         |
|   FAST    |        17874         |
|   BRISK   |        27116         |
|    ORB    |         5000         |
|   AKAZE   |        13429         |
|   SIFT    |        13860         |



#### Number of Matched Key-points for 10 Images

| Detectors\Descriptors | BRISK |  BRIEF  |      ORB      | FREAK | AKAZE | SIFT |
| :-------------------: | :---: | :-----: | :-----------: | :---: | :---: | :--: |
|       SHITOMASI       |  224  |   288 |      272      |  224  |  N/A  | N/A  |
|        HARRIS         |  123  |   185   |      155      |  126  |  N/A  | N/A  |
|         FAST          |  137  |   138   |      118      |  108  |  N/A  | N/A |
|         BRISK         |  182  |   241   |      170      |  111  |  N/A  | N/A  |
|          ORB          |  262  | **289**   |      235      |  225  |  N/A  | N/A  |
|         AKAZE         |  192  |   255   |      199      |  195  |  203  | N/A  |
|         SIFT          |  130  |   147   | Out of Memory |  108  |  N/A  | N/A  |

* KAZE/AKAZE descriptors will only work with KAZE/AKAZE keypoints.(It is because AKAZE fill the keypoints with one additional field which can only be extracted by appropriate descriptor and that is AKAZE)
* SIFT can be a detector but not the descriptor.
* SHITOMASI with blockSize = 4
* HARRIS with blockSize = 2
* In the matching step, the BF approach is used with the descriptor distance ratio set to 0.8.



#### Key-point Detection and Descriptor Extraction Time Consumption (in ms)

| Detectors\Descriptors |  BRISK  |    BRIEF    |      ORB      |  FREAK  |  AKAZE  |    SIFT    |
| :-------------------: | :-----: | :---------: | :-----------: | :-----: | :-----: | :--------: |
|       SHITOMASI       | 440.179 |   477.659   |    757.548    | 817.08 |   N/A   |  N/A  |
|        HARRIS         | 424.439 |   517.288   |    762.265    | 817.475 |   N/A   | N/A   |
|         FAST          | **73.6087** | 98.7083 |    376.191    | 478.229 |   N/A   |  N/A  |
|         BRISK         | 721.435 |   723.216    |   989.417    | 1065.08 |   N/A   | N/A |
|          ORB          | 578.056 |   656.15   |     901.194    | 990.528 |   N/A   | N/A  |
|         AKAZE         | 1772.78 |   1757.16   |    2034.87    | 2072.82 | 3348.77 | N/A   |
|         SIFT          | 807.395 |   831.656    | Out of Memory | 1135.71 |   N/A   |  N/A   |

#### Efficiency (matches/ms)

| Detectors\Descriptors |  BRISK   |    BRIEF    |      ORB      |  FREAK   |  AKAZE   |   SIFT   |
| :-------------------: | :------: | :---------: | :-----------: | :------: | :------: | :------: |
|       SHITOMASI       | 0.508884 |   0.602941    |    0.359053    | 0.274147  |   N/A    | N/A  |
|        HARRIS         | 0.289794  |  0.357635   |     0.203341    | 0.154133 |   N/A    |  N/A  |
|         FAST          | **1.86119**  | 1.39806 |    0.313671    |  0.225833  |   N/A    | N/A  |
|         BRISK         | 0.252275  |   0.333234   |    0.171818    | 0.104217 |   N/A    | N/A |
|          ORB          | 0.453243  |   0.44044   |     0.260765    | 0.227152  |   N/A    | N/A  |
|         AKAZE         | 0.108305 |    0.145121   |    0.0977952    | 0.0940746  | 0.0606193 | N/A |
|         SIFT          | 0.161012 |    0.176756   | Out of Memory | 0.095095 |   N/A    | N/A |



## TOP3 detector / descriptor combinations

1. FAST + BRISK
2. FAST + BRIEF
3. SHITOMASI + BRIEF


## Reference

* [KAZE/AKAZE descriptors will only work with KAZE/AKAZE keypoints](<https://github.com/kyamagu/mexopencv/issues/351>)
* https://github.com/FrancoisMasson1990/SFND_2D_Feature_Tracking
* https://github.com/gpokhark/SFND_2D_Feature_Matching


