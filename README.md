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

1. The Data Buffer, implementing a ring buffer with a fixed maximum size of 2, as this is done to make sure that the latest two consecutive frames are stored in a vector of  DataFrame structure type and older data is erased out. 

   

   > So in order to implement this, a condition is applied while pushing a new DataFrame coming from the new image; If the size of the databuffer increases above the maximum size of the ring buffer that is 2 in this case, the older data is erased by calling erase at the begining of the databuffer vector.

   

2. Detectors producing keypoints. Following detectors are readly available in the Opencv : HARRIS, SHITOMASI, FAST, BRISK, ORB, AKAZE, SIFT. Implementing them as switch or if else conditions where changing the detector type string can be changed and it calls the corresponding detector.

   

   >  There are three functions to implemented and create the above mentioned detectors ``` double detKeypointsShiTomasi(vector<cv::KeyPoint> &keypoints, cv::Mat &img)``` , ``` double detKeypointsHarris(std::vector<cv::KeyPoint> &keypoints, cv::Mat &img) ``` and ``` double detKeypointsModern(std::vector<cv::KeyPoint> &keypoints, cv::Mat &img, std::string detectorType) ``` ; detKeypointsModern function returns the corresponding detector type which is passed as one of the input argument. ```detect``` method is called on these detector objects and time of detection is returned by the functions; It is used for the performance metrics.

   

3. Filtering the keypoints, so that only keypoints from the region of interest could be retained.

   

   > In order to focus on the keypoints around the preceding vehicle, a rectangular region is considered where the vehicle lies throughout the 10 frame sequence. An Opencv technique rect's ```contains``` method is used to filter all the keypoints lying outside the rectangle .

   

4.Descriptors extracting keypoints and storing them in structures optimized for comparison between two consecutive frames:  Following descriptors are readly available in the Opencv :BRIEF, ORB, FREAK, AKAZE and SIFT and Implementing them as switch or if else conditions where changing the descriptor type string can be changed and it calls the corresponding descriptor.

   

   > For Descriptor extractors, compute is called on the keypoints those are obtained from detectors. The time of compute is returned by the function call; It is used for the performance metrics

   

5. Matching the keypoints stored as special descriptor structures to attain the associativity between two Mono camera frames in time: FLANN matching was to be implemented in addition to the pre-existing Brute Force matching. Then comes the selector types Nearest Neighbour or  k-nearest neighbor . These could be selected by changing the string selector type in the main file ```MidTermProject_Camera_Student.cpp```


   

6. Descriptor Distance Ratio: Use the K-Nearest-Neighbor matching to implement the descriptor distance ratio test, which looks at the ratio of best vs. second-best match to decide whether to keep an associated pair of keypoints.

   

   >Currently both nearest neighbour(SEL_NN) and K nearest neighbour(SEL_KNN) are implemented. For the performance analysis K value used is 2. And the Descriptor Distance Ratio for filtering the outliers from SEL_KNN only the top two match pairs adhering to the condition ``` knn_match1.distance < 0.8 knn_match2.distance ```
  
   

7. First Performance Evaluation Criteria: Count number of keypoints detected by the detector for all 10 frames present in the sequence
   

8. Second Performance Evaluation Criteria: Count number of matched keypoints detected by the brute force matching with the K nearest neighbour selector where k is set to 2 and Descriptor Distance Ratio set to 0.8.


   

9. Third Performance Evaluation Criteria: Measure the total time taken by the detection and extraction of the keypoints which includes time taken by the Detector to detect the keypoints in a frame and time taken by the Descriptor to extract the keypoints. Finally select TOP three detector and descriptor combination from the readings based on the parameter ratio: (Number of Mathes/Total time consumed)

   



## Performance Metrics



#### Total Number of Key-points detected and corresponding Matches for all 10 Images

| Detectors\Descriptors |  Number of Key-points | BRISK  |    BRIEF    |      ORB      |  FREAK  |  AKAZE  |    SIFT    |
| :-------------------: |  :------------------: |:-----: | :---------: | :-----------: | :-----: | :-----: | :--------: |
|        SHITOMASI     |      13423           |  224  |   288 |      272      |  224  |  N/A  | N/A  |
|        HARRIS        |      1737            |  123  |   185   |      155      |  126  |  N/A  | N/A  |
|         FAST         |      17874           |  137  |   138   |      118      |  108  |  N/A  | N/A |
|         BRISK        |      27116           |  182  |   241   |      170      |  111  |  N/A  | N/A  |
|          ORB         |      5000            |  262  | **289**   |      235      |  225  |  N/A  | N/A  |
|         AKAZE        |      13429           |  192  |   255   |      199      |  195  |  203  | N/A  |
|         SIFT         |      13860           |  130  |   147   | Ran out of memory |  108  |  N/A  | N/A  |

* AKAZE can only run as descriptor with AKAZE as detector because.(AKAZE fill the keypoints with one additional field which can only be extracted by appropriate descriptor and that is AKAZE)
* SIFT can be a detector but not the descriptor.
* HARRIS detector is used with a blockSize of 2
* SHITOMASI detector is used with a blockSize of 4.



#### Total time it took to run both Detector and Descriptor in milliseconds

| Detectors & Descriptors |  BRISK  |    BRIEF    |      ORB      |  FREAK  |  AKAZE  |    SIFT    |
| :-------------------: | :-----: | :---------: | :-----------: | :-----: | :-----: | :--------: |
|       SHITOMASI       | 440.179 |   477.659   |    757.548    | 817.08 |   N/A   |  N/A  |
|        HARRIS         | 424.439 |   517.288   |    762.265    | 817.475 |   N/A   | N/A   |
|         FAST          | **73.6087** | 98.7083 |    376.191    | 478.229 |   N/A   |  N/A  |
|         BRISK         | 721.435 |   723.216    |   989.417    | 1065.08 |   N/A   | N/A |
|          ORB          | 578.056 |   656.15   |     901.194    | 990.528 |   N/A   | N/A  |
|         AKAZE         | 1772.78 |   1757.16   |    2034.87    | 2072.82 | 3348.77 | N/A   |
|         SIFT          | 807.395 |   831.656    | Ran out of memory | 1135.71 |   N/A   |  N/A   |

#### Performance or ranking criteria Ratio which is number of matches/ total time consumed in ms

| Detectors & Descriptors |  BRISK   |    BRIEF    |      ORB      |  FREAK   |  AKAZE   |   SIFT   |
| :-------------------: | :------: | :---------: | :-----------: | :------: | :------: | :------: |
|       SHITOMASI       | 0.508884 |   0.602941    |    0.359053    | 0.274147  |   N/A    | N/A  |
|        HARRIS         | 0.289794  |  0.357635   |     0.203341    | 0.154133 |   N/A    |  N/A  |
|         FAST          | **1.86119**  | 1.39806 |    0.313671    |  0.225833  |   N/A    | N/A  |
|         BRISK         | 0.252275  |   0.333234   |    0.171818    | 0.104217 |   N/A    | N/A |
|          ORB          | 0.453243  |   0.44044   |     0.260765    | 0.227152  |   N/A    | N/A  |
|         AKAZE         | 0.108305 |    0.145121   |    0.0977952    | 0.0940746  | 0.0606193 | N/A |
|         SIFT          | 0.161012 |    0.176756   | Ran out of memory | 0.095095 |   N/A    | N/A |



## Based on the Ratio TOP3 detector and descriptor combinations are:

1. FAST + BRISK
2. FAST + BRIEF
3. SHITOMASI + BRIEF





## References

* KAZE/AKAZE descriptors will only work with KAZE/AKAZE keypoints
