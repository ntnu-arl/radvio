# RadVIO
Tightly-Coupled <b>Rad</b>ar-<b>V</b>isual-<b>I</b>nertial <b>O</b>dometry

[![DOI](https://img.shields.io/badge/DOI-10.1016/j.ejcon.2026.101601-blue)](https://doi.org/10.1016/j.ejcon.2026.101601)
[![YouTube](https://img.shields.io/badge/YouTube-KP5RGExULNM-red)](https://youtu.be/KP5RGExULNM)

This repository contains the implementation of the work [Tightly-Coupled Radar-Visual-Inertial Odometry](https://doi.org/10.1016/j.ejcon.2026.101601), published in the European Journal of Control. The method augments visual-inertial odometry (VIO) to showcase more robust performance in environments with visual degradation through radar fusion (via Doppler updates and radar-based feature depth initialization). The method can operate in either vision- or radar-only modes, thus making it also robust to temporary dropout from either exteroceptive sensor. This method is developed over ROVIO ([IROS 2015](http://dx.doi.org/10.3929/ethz-a-010566547), [IJRR 2017](http://dx.doi.org/10.1177/0278364917728574)).

For more information, please see our [publication](https://doi.org/10.1016/j.ejcon.2026.101601).

[![Video Title Screen](https://img.youtube.com/vi/KP5RGExULNM/maxresdefault.jpg)](https://www.youtube.com/watch?v=KP5RGExULNM)

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

## Data

The datasets used in this paper can be found [here](https://huggingface.co/datasets/ntnu-arl/radvio_dataset).

## Acknowledgements

* We thank M. Bloesch, M. Burri, S. Omari, M. Hutter, and R. Siegwart for ROVIO ([IROS 2015](http://dx.doi.org/10.3929/ethz-a-010566547), [IJRR 2017](http://dx.doi.org/10.1177/0278364917728574)), upon which this work is based.
* We thank Nikhil Khedekar for initial support with voxel mapping.

## Reference

If you use any of the implementation or data in your research, please cite the following publication:

```bibtex
@article{nissov2026radvio,
  title   = {Tightly-coupled radar-visual-inertial odometry},
  author  = {Morten Nissov and Mohit Singh and Kostas Alexis},
  journal = {European Journal of Control},
  pages   = {101601},
  year    = {2026},
  issn    = {0947-3580},
  doi     = {10.1016/j.ejcon.2026.101601},
  url     = {https://www.sciencedirect.com/science/article/pii/S0947358026001548}
}
```
