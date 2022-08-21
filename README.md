
&nbsp;

###  What is MRCV?

___Mixed Reality Computer Vision___ is an open-source computer vision library that abstracts the [SLAM](https://en.wikipedia.org/wiki/Simultaneous_localization_and_mapping) / geometrical environment and position tracking for DIY HMDs developers and enthusiasts, allowing easy implementations of AR and VR apps on cheap hardware. The goal is to run on the [Raspberry Pi Zero 2](https://www.raspberrypi.com/products/raspberry-pi-zero-2-w/), a $15 dollar 5 GFLOP tiny computer.

For now, we makes use of the existing [OpenCV](https://opencv.org/) library.

<!-- ![ezgif-2-ae8c50b6ee](https://user-images.githubusercontent.com/58654842/151255597-4dbf32bd-860d-4687-9841-5757a74ef90f.gif) -->

&nbsp;

#### Requires:
* OpenGL, [OpenCV](https://opencv.org/)
* [Pangolin](https://github.com/stevenlovegrove/Pangolin) ( optional for visualisation )
* [Eigen](https://eigen.tuxfamily.org/index.php?title=Main_Page)

&nbsp;

#### Running the demo on a local machine.
```bash
$ mkdir build && cd build
$ cmake ..
$ cmake --build .
```
&nbsp;

#### Usage
It can either run from a live video feed or a local video.

&nbsp;

Configuration macros:
```c++
#define MRCV_DEBUG                               // Run SLAM in debug mode.
#define MRCV_GRAPHING                            // Plot displacement, velocity and performance
#define RANSAC_THRESH [ threshold ]              // RANSAC threshold, defaulted to 3
#define RANSAC_MAXITERS [ number of iterations ] // RANSAC maximum iterations, defaulted to 2000
#define NO_EKF                                   // Disable the Extended Kalman Filter
```

&nbsp;


Include the main library file. This will auto-configure internal settings depending on what imports are available to it. Two linear algebra backends are available, Eigen or OpenCV. The Extended Kalman Filter library is dependent on Eigen and will not run if it is not available. If Pangolin is not available, only video and keypoint visualisation will be available, without a 3D interactive map of the resulting scan.
```c++
#include <Slam.h>
```

&nbsp;

First, you must define a camera calibration object that contains specific information about your camera:

```c++
struct CameraDetails {
	size_t frame_width;
	size_t frame_height;
	float sensor_width;
	float sensor_height	= 0.f;
	float focal_scaling = 1.f;
	size_t row_step = 5;
	size_t col_step = 5;
	size_t principal_x = frame_width / 2;
	size_t principal_y = frame_height / 2;
};
```
```c++
// Example use:
CameraDetails calibration;
calibration.frame_width = 1280;
calibration.frame_height = 720;
calibration.sensor_width = 5.76f; // Samsung Note 10 Plus camera
```

and give this object as an argument to the SLAM constructor, together with the path of the video:
```c++
Slam( std::string video_path, CameraDetails cam_details );
```

&nbsp;

Otherwise, all of these values can be inserted directly into the constructor:
```c++
Slam( std::string video_path, size_t frame_width, size_t frame_height, size_t row_step, 
	size_t col_step float Fx_cm, float Fy_cm, float sensor_width, float sensor_height, 
	float focal_scaling, size_t ransac_cycles, size_t ransac_thresh );
```

### Debug

To enter debug mode, define the `MRCV_DEBUG` macro:
```c++
#define MRCV_DEBUG
```
