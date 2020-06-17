[![Udacity - Robotics NanoDegree Program](https://s3-us-west-1.amazonaws.com/udacity-robotics/Extra+Images/RoboND_flag.png)](https://www.udacity.com/course/sensor-fusion-engineer-nanodegree--nd313)

# Udacity Nanodegree: Sensor Fusion

## Project 02: 2D Feature Tracking

<img src="Project/images/keypoints.png" width="820" height="248" />

The goal of this project is to develop a software stack using the OpenCV library, which can track the features in the successive frames coming from the camera of the car. In this project, I have explored different kinds of Feature Detectors, Descriptors, and Matching algorithms. Eventually, I will also do a comprehensive analysis of Descriptors and Feature Detectors based on performance(time), quality(robustness), and quantity of features. The high level of understanding of the project is as follows.


```
1 First, we will load images and store in ring buffer fashion using data structure which is optimal for the task
2 Then, we will do feature extraction using SHITOMASI, HARRIS, FAST, BRISK, ORB, AKAZE, and SIFT
3 The next part is about descriptor extraction which we will carry out using BRIEF, BRISK, ORB, FREAK, and SIFT
4 Fourth step is to do matching using brute force and also the FLANN approach 
5 Finally, we will do the analysis of different descriptor, detector and matching algorithms 
```

![alt txt](/Project/images/Matching.png)


### Dependencies for Running Locally
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


### Basic Build Instructions

#### 1. First of all, clone this repo:
```
$ git clone git@github.com:Suraj0712/SFND_2_Camera_Based_2D_Tracking.git
```

#### 2. Run Quiz
```
$ cd <Path_to_quiz_directory>
$ makdir build && cd build
$ cmake ..
$ make
$ ./<executable_name>
```
#### 3. Run Project
```
$ cd <directory_where_you_have_cloned_the_repo>/SFND_2_Camera_Based_2D_Tracking/Project/
$ makdir build && cd build
$ cmake ..
$ make
$ ./<executable_name>
```


### Directory Structure
```
.SFND_2_Camera_Based_2D_Tracking
├── LICENSE
├── Project						#Project directory
│   ├── CMakeLists.txt
│   ├── images
│   │   ├── keypoints.png
│   │   ├── KITTI
│   │   │   └── 2011_09_26
│   │   │       └── image_00
│   │   │           └── data
│   │   │               └── 0000000009.png
│   │   └── Matching.png
│   ├── Observations					#Directory with analysis results
│   │   ├── 7_DetectorVsKeypoints.csv
│   │   ├── 8_Detector_Descriptor_Matches.csv
│   │   └── 9_DetectorDescriptorTime.csv
│   └── src						#Source code directory
│       ├── dataStructures.h
│       ├── matching2D.hpp
│       ├── matching2D_Student.cpp
│       └── MidTermProject_Camera_Student.cpp
├── quiz						#Quiz assignments
│   ├── 1_OpenCV_exercises
│   │   └── OpenCV_exercises
│   │       ├── CMakeLists.txt
│   │       ├── images
│   │       │   └── ...
│   │       └── src
│   │           ├── change_pixels.cpp
│   │           ├── create_matrix.cpp
│   │           ├── load_image_1.cpp
│   │           ├── load_image_2.cpp
│   │           └── load_image_3.cpp
│   ├── 2_TTC_lidar
│   │   ├── CMakeLists.txt
│   │   ├── dat
│   │   │   ├── C22A5_currLidarPts.dat
│   │   │   └── C22A5_prevLidarPts.dat
│   │   └── src
│   │       ├── compute_ttc_lidar.cpp
│   │       ├── dataStructures.h
│   │       ├── structIO.cpp
│   │       └── structIO.hpp
│   ├── 3_TTC_camera
│   │   ├── CMakeLists.txt
│   │   ├── dat
│   │   │   ├── C23A5_KptMatches_AKAZE.dat
│   │   │   ├── C23A5_KptMatches_SHI-BRISK.dat
│   │   │   ├── C23A5_KptsRef_AKAZE.dat
│   │   │   ├── C23A5_KptsRef_SHI-BRISK.dat
│   │   │   ├── C23A5_KptsSource_AKAZE.dat
│   │   │   └── C23A5_KptsSource_SHI-BRISK.dat
│   │   └── src
│   │       ├── compute_ttc_camera.cpp
│   │       ├── dataStructures.h
│   │       ├── structIO.cpp
│   │       └── structIO.hpp
│   ├── 4_gradient_filtering
│   │   ├── CMakeLists.txt
│   │   ├── images
│   │   │   └── ...
│   │   └── src
│   │       ├── gaussian_smoothing.cpp
│   │       ├── gradient_sobel.cpp
│   │       └── magnitude_sobel.cpp
│   ├── 5_cornerness_harris
│   │   ├── CMakeLists.txt
│   │   ├── images
│   │   │   └── ...
│   │   └── src
│   │       └── cornerness_harris.cpp
│   ├── 6_detect_keypoints
│   │   ├── CMakeLists.txt
│   │   ├── images
│   │   │   └── ...
│   │   └── src
│   │       └── detect_keypoints.cpp
│   ├── 7_describe_keypoints
│   │   ├── CMakeLists.txt
│   │   ├── images
│   │   │   └── ...
│   │   └── src
│   │       └── describe_keypoints.cpp
│   └── 8_descriptor_matching
│       ├── CMakeLists.txt
│       ├── dat
│       │   └── ...
│       ├── images
│       │   └── ...
│       └── src
│           ├── dataStructures.h
│           ├── descriptor_matching.cpp
│           ├── structIO.cpp
│           └── structIO.hpp
└── README.md
```

### Project Rubric

#### Data Buffer

##### MP.1 Data Buffer Optimization

   > Data Buffer implemented using ```std:: deque``` which allows insertion and deletion at either end in O(1). As the required data structure should follow the FIFO approach ```std:: queue``` seems the best option, However lack of iterative make this option second to ```std:: deque```. 


#### Keypoints

##### MP.2 Keypoint Detection

   > Based on the input string, we controlled the program flow using  ```if..else if..else``` statements. To detect the features detKeypointsShiTomasi(for Shi-Thomasi), detKeypointsHarris(for Harris corner), detKeypointsModern(for FAST, BRISK, ORB, AKAZE, and SIFT) were implemented. The track of time is logged for performance evaluation.

##### MP.3 Keypoint Removal

   > As the region of interest is only the car in front of our ego car we will filter all the features and only keep the features belongs to the car using a predefined box. this will also help us in reducing the noise and save computation power for the following steps.


#### Descriptors

##### MP.4 Keypoint Descriptors

   > Descriptor is a vector of values, which describes the image patch around a key point. Similar to step 2, we implement a function to detection descriptors based on the input string. We still use OpenCV build-in descriptors (BRIEF, ORB, FREAK, AKAZE, and SIFT) class with default parameters to uniquely identify key points. Similar to step 2, we log the descriptor extraction time for performance evaluation.

##### MP.5 Descriptor Matching

   >We integrated OpenCV's Brute-force matcher(MAT_BF) and FLANN matcher(MAT_FLANN) algorithms for matching. User can choose either algorithm for feature matching however special care needs to be taken while giving the distance matrix for the Brute-force matcher (Hamming distance-> binary descriptors, L2 norm-> for HOG based ones)

##### MP.6 Descriptor Distance Ratio

   >Both nearest neighbor (best match) and K nearest neighbors (default k=2) are implemented. For KNN matching, we filter matches using the descriptor distance ratio test to remove some outliers.KNN looks at the ratio of best vs. second-best match to decide whether to keep an associated pair of key points.


#### Performance

##### MP.7 Performance Evaluation 1

   >Count the number of key points on the preceding vehicle for all 10 images and take note of the distribution of their neighborhood size. Do this for all the detectors you have implemented.

##### MP.8 Performance Evaluation 2

   >Count the number of matched keypoints for all 10 images using all possible combinations of detectors and descriptors. In the matching step, the BF approach is used with the descriptor distance ratio set to 0.8.

##### MP.9 Performance Evaluation 3

   >Log the time it takes for keypoint detection and descriptor extraction. The results must be entered into a spreadsheet and based on this data, the TOP3 detector/descriptor combinations must be recommended as the best choice for our purpose of detecting keypoints on vehicles.


### Benchmark

#### 1. Number of keypoints detected in images

[Click here to see the complete results](/Project/Observations/7_DetectorVsKeypoints.csv)

|Detector |Image1|Image2|Image3|Image4|Image5|Image6|Image7|Image8|Image9|Image10|
|---------|------|------|------|------|------|------|------|------|------|-------|
|SHITOMASI| 125  |118   |123   |120   |120   |113   |114   |123   |111   |112    |
|HARRIS   | 17   |14    |18    |21    |26    |43    |18    |31    |26    |34     |
|FAST     | 419  |427   |404   |423   |386   |414   |418   |406   |396   |401    |
|BRISK    | 254  |274   |276   |275   |293   |275   |289   |268   |259   |250    |
|ORB      | 91   |102   |106   |113   |109   |124   |129   |127   |124   |125    |
|AKAZE    | 162  |157   |159   |154   |162   |163   |173   |175   |175   |175    |
|SIFT     | 138  |132   |124   |137   |134   |140   |137   |148   |159   |137    |

#### 2. Number of keypoint matches in succesive frames

[Click here to see the complete results](/Project/Observations/8_Detector_Descriptor_Matches.csv)

|Detector |Descriptor|Img1-Img2 |Img2-Img3       |Img3-Img4|Img4-Img5|Img5-Img6|
|---------|----------|----------|----------------|---------|---------|---------|
|SHITOMASI|BRISK     |84        |80              |73       |77       |74       |
|SHITOMASI|BRIEF     |96        |93              |92       |89       |92       |
|SHITOMASI|ORB       |86        |84              |87       |91       |87       |
|SHITOMASI|FREAK     |66        |66              |64       |63       |62       |
|SHITOMASI|SIFT      |96        |86              |83       |91       |86       |
|HARRIS   |BRISK     |11        |9               |10       |11       |16       |
|HARRIS   |BRIEF     |12        |12              |14       |17       |17       |
|HARRIS   |ORB       |11        |11              |14       |17       |19       |
|HARRIS   |FREAK     |11        |9               |13       |14       |13       |
|HARRIS   |SIFT      |15        |12              |15       |18       |22       |
|FAST     |BRISK     |213       |216             |187      |205      |185      |
|FAST     |BRIEF     |229       |253             |233      |247      |224      |
|FAST     |ORB       |226       |220             |218      |226      |220      |
|FAST     |FREAK     |178       |181             |156      |182      |159      |
|FAST     |SIFT      |257       |246             |237      |236      |232      |
|BRISK    |BRISK     |125       |143             |129      |138      |108      |
|BRISK    |BRIEF     |138       |166             |129      |141      |148      |
|BRISK    |ORB       |132       |146             |122      |144      |130      |
|BRISK    |FREAK     |95        |111             |91       |102      |95       |
|BRISK    |SIFT      |137       |151             |145      |146      |144      |
|ORB      |BRISK     |36        |37              |36       |43       |40       |
|ORB      |BRIEF     |37        |38              |37       |53       |42       |
|ORB      |ORB       |42        |39              |55       |44       |39       |
|ORB      |FREAK     |30        |30              |41       |33       |33       |
|ORB      |SIFT      |51        |49              |51       |48       |50       |
|AKAZE    |BRISK     |118       |109             |112      |110      |109      |
|AKAZE    |BRIEF     |108       |116             |110      |109      |116      |
|AKAZE    |ORB       |113       |106             |108      |110      |117      |
|AKAZE    |FREAK     |104       |98              |91       |87       |87       |
|AKAZE    |SIFT      |130       |116             |102      |109      |111      |
|SIFT     |BRISK     |55        |60              |58       |60       |52       |
|SIFT     |BRIEF     |63        |72              |64       |66       |52       |
|SIFT     |ORB       |63        |57              |61       |61       |52       |
|SIFT     |FREAK     |61        |62              |55       |63       |49       |
|SIFT     |SIFT      |75        |72              |65       |75       |59       |

#### 3. Key-point Detection and Descriptor Extraction Time Consumption (in ms)

[Click here to see the complete results](/Project/Observations/9_DetectorDescriptorTime.csv)

|Detector |Descriptor|Average_det_desc|Img2_det        |Img2_des|det_des_2|Img3_det |Img3_des|det_des_3|
|---------|----------|----------------|----------------|--------|---------|---------|--------|---------|
|SHITOMASI|BRISK     |12.285          |10.2517         |1.5117  |11.7634  |22.9957  |1.1704  |24.1661  |
|SHITOMASI|BRIEF     |19.2929         |10.175          |0.8587  |11.0337  |26.5259  |0.6042  |27.1301  |
|SHITOMASI|ORB       |18.3238         |10.2195         |2.7998  |13.0193  |9.4221   |2.6874  |12.1095  |
|SHITOMASI|FREAK     |50.9559         |9.7698          |33.2135 |42.9833  |8.3841   |30.6035 |38.9876  |
|SHITOMASI|SIFT      |30.8783         |11.8756         |18.4094 |30.285   |11.7854  |17.8116 |29.597   |
|HARRIS   |BRISK     |12.9866         |9.8407          |0.7617  |10.6024  |23.0235  |0.3903  |23.4138  |
|HARRIS   |BRIEF     |19.4674         |9.7881          |0.6074  |10.3955  |9.7469   |0.1699  |9.9168   |
|HARRIS   |ORB       |23.0883         |9.5962          |2.4669  |12.0631  |9.0491   |2.314   |11.363   |
|HARRIS   |FREAK     |43.2029         |9.244           |29.9931 |39.2371  |9.4155   |30.8532 |40.2687  |
|HARRIS   |SIFT      |43.2568         |17.5699         |23.4894 |41.0593  |17.7283  |23.3189 |41.0472  |
|FAST     |BRISK     |4.6206          |1.337           |3.562   |4.899    |1.188    |3.5807  |4.7687   |
|FAST     |BRIEF     |4.4616          |1.2812          |1.4501  |2.7312   |1.6517   |1.8108  |3.4625   |
|FAST     |ORB       |6.457           |1.3606          |3.1618  |4.5223   |5.2329   |3.3195  |8.5524   |
|FAST     |FREAK     |37.4645         |1.2863          |34.4573 |35.7436  |5.1419   |36.5957 |41.7376  |
|FAST     |SIFT      |57.9681         |2.1376          |57.2154 |59.353   |1.9701   |54.8043 |56.7744  |
|BRISK    |BRISK     |31.3316         |30.0693         |2.3689  |32.4382  |28.8759  |2.3451  |31.221   |
|BRISK    |BRIEF     |34.7142         |30.1886         |0.7111  |30.8997  |30.2851  |0.7137  |30.9988  |
|BRISK    |ORB       |38.1599         |29.7682         |8.8373  |38.6055  |29.1505  |8.6992  |37.8497  |
|BRISK    |FREAK     |61.6631         |29.9094         |34.5003 |64.4097  |28.4116  |33.0934 |61.505   |
|BRISK    |SIFT      |112.6024        |43.8681         |64.5253 |108.3934 |46.7225  |67.171  |113.8935 |
|ORB      |BRISK     |14.7223         |5.0463          |1.0179  |6.0642   |16.3922  |1.0235  |17.4157  |
|ORB      |BRIEF     |20.1527         |5.0697          |0.297   |5.3668   |16.3395  |0.7428  |17.0823  |
|ORB      |ORB       |29.9825         |4.7644          |9.1361  |13.9004  |4.8181   |8.6071  |13.4252  |
|ORB      |FREAK     |52.9391         |4.9399          |32.6276 |37.5675  |16.8525  |40.4225 |57.275   |
|ORB      |SIFT      |87.1262         |8.112           |68.0712 |76.1832  |8.3619   |74.6091 |82.971   |
|AKAZE    |BRISK     |44.0101         |41.407          |1.6844  |43.0914  |40.2673  |1.4472  |41.7145  |
|AKAZE    |BRIEF     |50.3566         |40.5057         |0.566   |41.0717  |60.3273  |0.889   |61.2163  |
|AKAZE    |ORB       |52.5394         |40.7551         |6.7143  |47.4694  |41.449   |5.7299  |47.1789  |
|AKAZE    |FREAK     |79.8823         |40.3914         |35.2877 |75.6791  |61.0671  |35.4387 |96.5058  |
|AKAZE    |SIFT      |137.4315        |100.037         |31.2121 |131.2491 |100.265  |31.7573 |132.0223 |
|SIFT     |BRISK     |156.5631        |164.297         |1.7352  |166.0322 |137.456  |1.6525  |139.1085 |
|SIFT     |BRIEF     |164.8741        |162.805         |0.7271  |163.5321 |160.865  |0.678   |161.543  |
|SIFT     |FREAK     |208.4729        |160.963         |45.4526 |206.4156 |160.396  |47.4581 |207.8541 |
|SIFT     |SIFT      |235.8312        |130.119         |97.1628 |227.2818 |131.569  |96.3781 |227.9471 |

#### 4. TOP3 detector/descriptor combinations

1. FAST + BRIEF
2. FAST + BRISK
3. FAST + ORB


## Thank you!!!


