# RadVIO
Tightly-Coupled <b>Rad</b>ar-<b>V</b>isual-<b>I</b>nertial <b>O</b>dometry

[![arXiv](https://img.shields.io/badge/arXiv-1234.56789-b31b1b.svg)](https://arxiv.org/abs/1234.56789)

This repository contains the implementation of the work [Tightly-Coupled Radar-Visual-Inertial Odometry](), accepted for publication at the 2026 European Control Conference (ECC). The method augments visual-inertial odometry (VIO) to showcase more robust performance in environments with visual degradation through the radar fusion (via Doppler updates and radar-based feature depth intialization). The method can operate in either vision- or radar-only modes, thus making it also robust to temporary dropout from either exteroceptive sensor. This method is developed over ROVIO ([IROS 2015](http://dx.doi.org/10.3929/ethz-a-010566547), [IJRR 2017](http://dx.doi.org/10.1177/0278364917728574)).

For more information, please see our [preprint]().

[![](https://img.youtube.com/vi/VIDEO_ID/maxresdefault.jpg)](https://www.youtube.com/watch?v=VIDEO_ID)

## Building

Assuming you have already installed `ros-noetic-desktop-full`, you can build RadVIO by

```bash
mkdir -p catkin_ws/src

cd catkin_ws
catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release

cd src
git clone https://github.com/ethz-asl/kindr.git
git clone https://github.com/ntnu-arl/radvio.git --recursive

rosdep install --from-paths src --ignore-src -r -y

catkin build radvio
```

## Usage Instructions

Exemplary launch files for running the node and running from the bag can be seen in `launch/`

### Parameters

* Camera matrix and distortion parameters should be provided by a yaml file
* Both camera and radar extrinsic transforms should be provided in `cfg/radvio.info`
  * For low-excitation trajectories, it can be helpful to disable the camera extrinsic estimation (`doVECalibration`)
  * The radar extrinsics can also be enabled/disabled (`doRECalibration`), however the method performance is sensitive to poor radar extrinsics
* Doppler measurement update (`DopplerUpdate`)
  * Assuming this is triggered, such that `chirp_duration_s` can be set to accurately reference the mid-chirp timestamp
* Radar depth initialization (`ImgUpdate/Radar`)
  * Depending on the density/accuracy of the radar point cloud, can be configured appropriately or disabled entirely (`doRadarInitialization`)
  * Performance can be sensitive to initialization covariance (`featureCovScaleFactor`)

## Acknowledgements

* We thank M. Bloesch for ROVIO ([IROS 2015](http://dx.doi.org/10.3929/ethz-a-010566547), [IJRR 2017](http://dx.doi.org/10.1177/0278364917728574)), upon which this work is based.
* We thank Nikhil Khedekar for initial support with voxel mapping.

## Reference

If you use this work in your research, please cite the following publication:

```bibtex
@misc{nissov2026radvio,
  arxiv
}
```
