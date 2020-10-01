/* INCLUDES FOR THIS PROJECT */
#include <iostream>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <vector>
#include <map>
#include <cmath>
#include <limits>
#include <opencv2/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <opencv2/xfeatures2d/nonfree.hpp>

#include "dataStructures.h"
#include "matching2D.hpp"

using namespace std;

enum detector1
{
    SHITOMASI,
    HARRIS
};

std::map<string, detector1> Levels;

void register1_levels()
{
    Levels["SHITOMASI"] = SHITOMASI;
    Levels["HARRIS"] = HARRIS;
}

/* MAIN PROGRAM */
int main(int argc, const char *argv[])
{

    /* INIT VARIABLES AND DATA STRUCTURES */

    // data location
    string dataPath = "../";

    // camera
    string imgBasePath = dataPath + "images/";
    string imgPrefix = "KITTI/2011_09_26/image_00/data/000000"; // left camera, color
    string imgFileType = ".png";
    int imgStartIndex = 0; // first file index to load (assumes Lidar and camera names have identical naming convention)
    int imgEndIndex = 18;   // last file index to load
    int imgFillWidth = 4;  // no. of digits which make up the file index (e.g. img-0001.png)

    // misc
    int dataBufferSize = 2;        // no. of images which are held in memory (ring buffer) at the same time
    vector<DataFrame> data_buffer; // list of data frames which are held in memory at the same time

    // for visualizing results
    bool bVis = true;
    bool bViskeyPoints = false;
    bool bFocusOnVehicle = true;
    bool bLimitKpts = true;

    double time_Total = 0;
    int Totalkeypoints = 0;
    int Totalmatches = 0;

    string detector_Type = "FAST"; // HARRIS, SHITOMASI, FAST, BRISK, ORB, AKAZE, SIFT
    string descriptor_Type = "BRIEF";   // BRISK, BRIEF, ORB, FREAK, AKAZE, SIFT

    register1_levels();
    /* MAIN LOOP OVER ALL IMAGES */

    for (size_t imgIndex = 0; imgIndex <= imgEndIndex - imgStartIndex; imgIndex++)
    {
        /* LOAD IMAGE INTO BUFFER */

        // assemble filenames for current index
        ostringstream imgNumber;
        imgNumber << setfill('0') << setw(imgFillWidth) << imgStartIndex + imgIndex;
        string imgFullFilename = imgBasePath + imgPrefix + imgNumber.str() + imgFileType;

        // load image from file and convert to grayscale
        cv::Mat img, imgGray;
        img = cv::imread(imgFullFilename);
        cv::cvtColor(img, imgGray, cv::COLOR_BGR2GRAY);

        //// STUDENT ASSIGNMENT
        //// TASK MP.1 -> replace the following code with ring buffer of size dataBufferSize

        // push image into data frame buffer
        DataFrame Frame;
        Frame.cameraImg = imgGray;
        data_buffer.push_back(Frame);

        if (data_buffer.size() > dataBufferSize)
        {
            data_buffer.erase(data_buffer.begin());
        }

        //// EOF STUDENT ASSIGNMENT
        cout << "#1 : LOAD IMAGE INTO BUFFER done" << endl;

        /* DETECT IMAGE KEYPOINTS */
        double time_Detect = 0;

        // extract 2D keypoints from current image
        vector<cv::KeyPoint> keypoints; // create empty feature list for current image

        //// STUDENT ASSIGNMENT
        //// TASK MP.2 -> add the following keypoint detectors in file matching2D.cpp and enable string-based selection based on detectorType
        if (Levels.find(detector_Type) != Levels.end())
        {
            switch (Levels[detector_Type])
            {

            case SHITOMASI:
                time_Detect = detKeypointsShiTomasi(keypoints, imgGray, false);
                break;
            case HARRIS:
                time_Detect = detKeypointsHarris(keypoints, imgGray, false);
                break;
            }
        }
        else
            time_Detect = detKeypointsModern(keypoints, imgGray, detector_Type, false);

        if (time_Detect == 0)
            return 1;

        Totalkeypoints += keypoints.size();

        //// EOF STUDENT ASSIGNMENT

        //// STUDENT ASSIGNMENT
        //// TASK MP.3 -> only keep keypoints on the preceding vehicle

        // only keep keypoints on the preceding vehicle

        cv::Rect vehicleRect(535, 180, 180, 150);
        if (bFocusOnVehicle)
        {
            // ...
            for (auto it = keypoints.begin(); it < keypoints.end(); it++)
            {

                // erase the keypoints those fall outside the area specified by the vehicleRect
                if (!vehicleRect.contains(it->pt))
                {
                    keypoints.erase(it);
                }
            }
        }

        //// EOF STUDENT ASSIGNMENT

        // optional : limit number of keypoints (helpful for debugging and learning)

        if (bLimitKpts)
        {   
            int maxKeypoints = 50;

            if (detector_Type.compare("SHITOMASI") == 0)
            { 
                // there is no response info, so keep the first 50 as they are sorted in descending quality order
                keypoints.erase(keypoints.begin() + maxKeypoints, keypoints.end());
            }
            cv::KeyPointsFilter::retainBest(keypoints, maxKeypoints);
            cout << " NOTE: Keypoints have been limited!" << endl;
        }

        // push keypoints and descriptor for current frame to end of data buffer
        (data_buffer.end() - 1)->keypoints = keypoints;
        cout << "#2 : DETECT KEYPOINTS done" << endl;

        /* EXTRACT KEYPOINT DESCRIPTORS */

        //// STUDENT ASSIGNMENT
        //// TASK MP.4 -> add the following descriptors in file matching2D.cpp and enable string-based selection based on descriptorType
        //// -> BRIEF, ORB, FREAK, AKAZE, SIFT

        cv::Mat descriptors;
        double time_Des = 0;

        time_Des = descKeypoints((data_buffer.end() - 1)->keypoints, (data_buffer.end() - 1)->cameraImg, descriptors, descriptor_Type);
        //// EOF STUDENT ASSIGNMENT

        if (time_Des == 0)
            return 2;

        // push descriptors for current frame to end of data buffer
        (data_buffer.end() - 1)->descriptors = descriptors;

        cout << "#3 : EXTRACT DESCRIPTORS done" << endl;

        if (data_buffer.size() > 1) // wait until at least two images have been processed
        {

            /* MATCH KEYPOINT DESCRIPTORS */

            vector<cv::DMatch> matches;
            string matcher_Type = "MAT_BF";               // MAT_BF, MAT_FLANN
            string descriptorTypeForMatch = "DES_BINARY"; // DES_BINARY, DES_HOG
            string selector_Type = "SEL_KNN";             // SEL_NN, SEL_KNN

            //// STUDENT ASSIGNMENT
            //// TASK MP.5 -> add FLANN matching in file matching2D.cpp
            //// TASK MP.6 -> add KNN match selection and perform descriptor distance ratio filtering with t=0.8 in file matching2D.cpp

            matchDescriptors((data_buffer.end() - 2)->keypoints, (data_buffer.end() - 1)->keypoints,
                             (data_buffer.end() - 2)->descriptors, (data_buffer.end() - 1)->descriptors,
                             matches, descriptorTypeForMatch, matcher_Type, selector_Type);

            Totalmatches += matches.size();
            //// EOF STUDENT ASSIGNMENT

            // store matches in current data frame
            (data_buffer.end() - 1)->kptMatches = matches;

            //cout << "#4 : MATCH KEYPOINT DESCRIPTORS done" << endl;

            // visualize matches between current and previous image

            if (bVis)
            {
                cv::Mat matchImg = ((data_buffer.end() - 1)->cameraImg).clone();
                cv::drawMatches((data_buffer.end() - 2)->cameraImg, (data_buffer.end() - 2)->keypoints,
                                (data_buffer.end() - 1)->cameraImg, (data_buffer.end() - 1)->keypoints,
                                matches, matchImg,
                                cv::Scalar::all(-1), cv::Scalar::all(-1),
                                vector<char>(), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);

                string windowName = "Matching keypoints between two camera images";
                cv::namedWindow(windowName, 7);
                cv::imshow(windowName, matchImg);
                cout << "Press key to continue to next image" << endl;
                cv::waitKey(0); // wait for key to be pressed
            }
        }
        time_Total += time_Detect + time_Des;

    } // eof loop over all images

    std::cout << std::endl;
    std::cout << "**************Performance*************" << std::endl;
    std::cout << detector_Type << " + " << descriptor_Type << std::endl;
    std::cout << "Number of Key-points for total frames= " << Totalkeypoints << std::endl;
    std::cout << "Number of Matches for total frames = " << Totalmatches << std::endl;
    std::cout << "Total time taken for detection and extraction = " << time_Total * 1000.0 << " ms" << std::endl;
    std::cout << "Performance Ratio = " << Totalmatches / (time_Total * 1000.0) << " matches/ms" << std::endl;
    std::cout << "**********************************" << std::endl;
    return 0;
}
