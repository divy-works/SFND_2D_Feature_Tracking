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


<h3>Project Specification</h3>
<h2>Camera Based 2D Feature Tracking</h2>
<div>
  <h3>Mid-Term Report</h3>
  <table>
    <thead>
      <tr>
        <td>CRITERIA</td>
        <td>SPECIFICATIONS</td>
        <td>How specifications was addressed?</td>
      </tr>
    </thead>
    <tbody>
      <tr>
        <td> MP.0 Mid-Term Project</td>
        <td> Provide a Writeup / README that includes all the rubric points and how you addressed each one. You can submit your writeup as markdown or pdf.</td>
        <td>
        <ul>
          <li>Tasks MP.7, MP.8, MP.9 are being addressed in reports/Camera_based_2D_Tracking_Report.pdf</li>
          <li>how other tasks has been addressed will be described in this README.md</li>
        </ul>
        </td>
      </tr>
    </tbody>
  </table>
</div>

<div>
  <h3>Data Buffer</h3>
  <table>
    <thead>
      <tr>
        <td>CRITERIA</td>
        <td>SPECIFICATIONS</td>
        <td>How specifications was addressed?</td>
      </tr>
    </thead>
    <tbody>
      <tr>
        <td> MP.1 Data Buffer Optimization</td>
        <td> Implement a vector for dataBuffer objects whose size does not exceed a limit (e.g. 2 elements). This can be achieved by pushing in new elements on one end and removing elements on the other end.</td>
        <td>implemented in lines from 64 to 83 src/MidTermProject_Camera_Student.cpp </td>
      </tr>
    </tbody>
  </table>
</div>

<div>
  <h3>Keypoints</h3>
  <table>
    <thead>
        <td>CRITERIA</td>
        <td>SPECIFICATIONS</td>
        <td>How specifications was addressed?</td>
      <tr>
    </thead>
    <tbody>
      <tr>
        <td> MP.2 Keypoint Detection</td>
        <td> Implement detectors HARRIS, FAST, BRISK, ORB, AKAZE, and SIFT and make them selectable by setting a string accordingly.</td>
        <td> implemented in lines from 97 to 112 of src/MidTermProject_Camera_Student.cpp and  in lines from 145 to 255 of src/matching2D_Student.cpp</td>
      </tr>
    </tbody>
    <tbody>
      <tr>
        <td> MP.3 Keypoint Removal</td>
        <td> Remove all keypoints outside of a pre-defined rectangle and only use the keypoints within the rectangle for further processing.</td>
        <td> implemented in lines from 118 to 134 of src/MidTermProject_Camera_Student.cpp </td>
      </tr>
    </tbody>
  </table>
</div>

<div>
  <h3>Descriptors</h3>
  <table>
    <thead>
        <td>CRITERIA</td>
        <td>SPECIFICATIONS</td>
        <td>How specifications was addressed?</td>
      <tr>
    </thead>
    <tbody>
      <tr>
        <td> MP.4 Keypoint Descriptors</td>
        <td> Implement descriptors BRIEF, ORB, FREAK, AKAZE and SIFT and make them selectable by setting a string accordingly. by setting a string accordingly.</td>
        <td> implemented  in lines from 162 to 165 of src/MidTermProject_Camera_Student.cpp and in lines from 73 to 99 of src/matching2D_Student.cpp</td>
      </tr>
    </tbody>
    <tbody>
      <tr>
        <td> MP.5 Descriptor Matching</td>
        <td> Implement FLANN matching as well as k-nearest neighbor selection. Both methods must be selectable using the respective strings in the main function.</td>
        <td> implemented from line 23 to 53 of src/matching2D_Student.cpp </td>
      </tr>
    </tbody>
    <tbody>
      <tr>
        <td> MP.6 Descriptor Distance Ratio</td>
        <td> Use the K-Nearest-Neighbor matching to implement the descriptor distance ratio test, which looks at the ratio of best vs. second-best match to decide whether to keep an associated pair of keypoints.</td>
        <td> implemented from lines 43 to 50 of src/matching2D_Student.cpp</td>
      </tr>
    </tbody>
  </table>
</div>

<div>
  <h3>Performance</h3>
  <table>
    <thead>
        <td>CRITERIA</td>
        <td>SPECIFICATIONS</td>
        <td>How specifications was addressed?</td>
      <tr>
    </thead>
    <tbody>
      <tr>
        <td> MP.7 Peformance Evaluation 1</td>
        <td> Count the number of keypoints on the preceding vehicle for all 10 images and take note of the distribution of their neighborhood size. Do this for all the detectors you have implemented.</td>
        <td> refer reports/Camera_based_2D_Tracking_Report.pdf </td>
      </tr>
    </tbody>
    <tbody>
      <tr>
        <td> MP.8 Performance Evaluation 2</td>
        <td> Count the number of matched keypoints for all 10 images using all possible combinations of detectors and descriptors. In the matching step, the BF approach is used with the descriptor distance ratio set to 0.8.</td>
        <td> refer reports/Camera_based_2D_Tracking_Report.pdf </td>
      </tr>
    </tbody>
    <tbody>
      <tr>
        <td> MP.9 Performance Evaluation 3</td>
        <td> Log the time it takes for keypoint detection and descriptor extraction. The results must be entered into a spreadsheet and based on this data, the TOP3 detector / descriptor combinations must be recommended as the best choice for our purpose of detecting keypoints on vehicles.</td>
        <td> refer reports/Camera_based_2D_Tracking_Report.pdf </td>
      </tr>
    </tbody>
  </table>
</div>