# 2D Feature Tracking

<img src="images/keypoints.png" width="820" height="248" />


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

## Code implementation
###  Data Buffer Optimization
For optimizing the memory load, ring buffer was implemented. The idea is to keep only a memory load of 2 images. Once image vector exceeds this limit the oldest image is remove from the vector.

```
if(dataBuffer.size()%(dataBufferSize+1)==0)
            dataBuffer.pop_back();
```

### Keypoint Detection

For the keypoints detection, 3 methods were implemented>
*  **detKeypointsShiTomasi** : keypoints detected by ShiTomasi algorithm
```
int blockSize = 4;       //  size of an average block for computing a derivative covariation matrix over each pixel neighborhood
    double maxOverlap = 0.0; // max. permissible overlap between two features in %
    double minDistance = (1.0 - maxOverlap) * blockSize;
    int maxCorners = img.rows * img.cols / max(1.0, minDistance); // max. num. of keypoints

    double qualityLevel = 0.01; // minimal accepted quality of image corners
    double k = 0.04;

    // Apply corner detection
    double t = (double)cv::getTickCount();
    vector<cv::Point2f> corners;
    cv::goodFeaturesToTrack(img, corners, maxCorners, qualityLevel, minDistance, cv::Mat(), blockSize, false, k);

    // add corners to result vector
    for (auto it = corners.begin(); it != corners.end(); ++it)
    {

        cv::KeyPoint newKeyPoint;
        newKeyPoint.pt = cv::Point2f((*it).x, (*it).y);
        newKeyPoint.size = blockSize;
        keypoints.push_back(newKeyPoint);
    }
    t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
    cout << "Shi-Tomasi detection with n=" << keypoints.size() << " keypoints in " << 1000 * t / 1.0 << " ms" << endl;
```
* **detKeypointsHarris**:keypoints detected by HARRIS algorithm

```
    double t = (double)cv::getTickCount();
    // Detector parameters
        int blockSize = 2;     // for every pixel, a blockSize × blockSize neighborhood is considered
        int apertureSize = 3;  // aperture parameter for Sobel operator (must be odd)
        int minResponse = 100; // minimum value for a corner in the 8bit scaled response matrix
        double k = 0.04;       // Harris parameter (see equation for details)
        cv::Mat dst, dst_norm, dst_norm_scaled;
        dst = cv::Mat::zeros(img.size(), CV_32FC1);
        cv::cornerHarris(img, dst, blockSize, apertureSize, k, cv::BORDER_DEFAULT);
        //normalize from 0 to 255 image dst
        cv::normalize(dst, dst_norm, 0, 255, cv::NORM_MINMAX, CV_32FC1, cv::Mat());

        cv::convertScaleAbs(dst_norm, dst_norm_scaled);
        // locate local maxima in the Harris response matrix 
        // Response matrix is a score for each local neighboor corner : det(M) - k*tr(M)^2
        // where k is the empirical harris parameter
        
        double maxOverlap = 0.0;
        for(int i=0; i<dst_norm.rows; i++){
            for(int j=0; j<dst_norm.cols; j++){
                int response = (int)dst_norm.at<float>(i,j);
                if(response> minResponse){
                     cv::KeyPoint newKeyPoint;
                     newKeyPoint.pt = cv::Point2f(j, i);
                     newKeyPoint.size = 2 * apertureSize;
                     newKeyPoint.response = response;
                     bool bOverlap = false;
                   
                    for (auto it = keypoints.begin(); it != keypoints.end(); ++it)
                {
                      double kptOverlap = cv::KeyPoint::overlap(newKeyPoint, *it);
                       if (kptOverlap > maxOverlap)
                    {
                        bOverlap = true;
                        if (newKeyPoint.response > (*it).response)
                        {                      // if overlap is >t AND response is higher for new kpt
                            *it = newKeyPoint; // replace old key point with new one
                            break;             // quit loop over keypoints
                        }

                    }
                }
                    if (!bOverlap)
                {                                     // only add new key point if no overlap has been found in previous NMS
                    keypoints.push_back(newKeyPoint); // store new keypoint in dynamic list
                }

            }
        }
    }
        t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
        cout << "HARRIS detection with n=" << keypoints.size() << " keypoints in " << 1000 * t / 1.0 << " ms" << endl;
```
* **detKeypointsModern**: implementation of others algorithm using if clauses (  FAST, BRISK, ORB, AKAZE, and SIFT)

```
int nb_detected_points=100;
    double t = (double)cv::getTickCount();
    if(detectorType.compare("FAST")== 0){
        cv::Ptr<cv::FastFeatureDetector> detector= cv::FastFeatureDetector::create(nb_detected_points, true);
        detector->detect(img, keypoints);
        t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
        cout << "FAST detection with n=" << keypoints.size() << " keypoints in " << 1000 * t / 1.0 << " ms" << endl;

    }
    
    if(detectorType.compare("BRISK")== 0){
        //Check parameters
        cv::Ptr<cv::BRISK> detector= cv::BRISK::create();
        detector->detect(img, keypoints);
        t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
        cout << "BRISK detection with n=" << keypoints.size() << " keypoints in " << 1000 * t / 1.0 << " ms" << endl;

    }
    if(detectorType.compare("ORB")== 0){
        cv::Ptr<cv::ORB> detector= cv::ORB::create(3000);
        detector->detect(img, keypoints);
        t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
        cout << "ORB detection with n=" << keypoints.size() << " keypoints in " << 1000 * t / 1.0 << " ms" << endl;

    }

    if(detectorType.compare("SIFT")== 0){
        cv::Ptr<cv::Feature2D> detector= SIFT::create();
        detector->detect(img, keypoints); 
        t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
        cout << "SIFT detection with n=" << keypoints.size() << " keypoints in " << 1000 * t / 1.0 << " ms" << endl;
  
    }
    if(detectorType.compare("AKAZE")== 0){
        cv::Ptr<cv::AKAZE> detector= cv::AKAZE::create();
        detector->detect(img, keypoints);
        t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
        cout << "AKAZE detection with n=" << keypoints.size() << " keypoints in " << 1000 * t / 1.0 << " ms" << endl;
    }
    
```

## Disclamer
This project was cloned from [Udacity 2D Feature tracking](https://github.com/udacity/SFND_2D_Feature_Tracking) in the context of [Sensor Fusion Engineer nanodegree](https://www.udacity.com/course/sensor-fusion-engineer-nanodegree--nd313).

