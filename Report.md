## Introduction
In this report, an analysis will be made to obtain the best image keypoints, descriptor generator and matcher. 
The keypoints generator used in this project are the following:
* HARRIS
* FAST 
* BRISK 
* ORB
* AKAZE
* SIFT
Regarding the descriptors, the analysis will be about the following ones:
* BRIEF
* ORB
* FREAK
* AKAZE 
* SIFT
Finally, the matchers used in this project are:
* FLANNBASED
* BRUTE FORCE pair matching

The performance analysis will be based on:
* time taken to compute keypoints and descriptors
* Number of points generated
* Number of matched KeyPoints ( i.e. true positive, false positive matches)

## Distribution of keypoints 

<img src="images/keypoints_dist.png" width="820" height="248" />
The distribution of keypoints generation above displayed shows that the algorithms with top number of points, either inside the region of interest (the car) either per the all frame, are BRISK and ORB. In addition, we can tell that the number of points generated per frame is quite stable per algorithm.  



