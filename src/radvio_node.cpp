/*
* Copyright (c) 2014, Autonomous Systems Lab
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
* * Redistributions of source code must retain the above copyright
* notice, this list of conditions and the following disclaimer.
* * Redistributions in binary form must reproduce the above copyright
* notice, this list of conditions and the following disclaimer in the
* documentation and/or other materials provided with the distribution.
* * Neither the name of the Autonomous Systems Lab, ETH Zurich nor the
* names of its contributors may be used to endorse or promote products
* derived from this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
* ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
* FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
* DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
*/


#include <memory>

#include <Eigen/StdVector>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#include <ros/ros.h>
#include <ros/package.h>
#include <geometry_msgs/Pose.h>
#pragma GCC diagnostic pop

#include "radvio/RadvioFilter.hpp"
#include "radvio/RadvioNode.hpp"
#ifdef MAKE_SCENE
#include "radvio/RadvioScene.hpp"
#endif

#ifdef RADVIO_NMAXFEATURE
static constexpr int nMax_ = RADVIO_NMAXFEATURE;
#else
static constexpr int nMax_ = 25; // Maximal number of considered features in the filter state.
#endif

#ifdef RADVIO_NLEVELS
static constexpr int nLevels_ = RADVIO_NLEVELS;
#else
static constexpr int nLevels_ = 4; // // Total number of pyramid levels considered.
#endif

#ifdef RADVIO_PATCHSIZE
static constexpr int patchSize_ = RADVIO_PATCHSIZE;
#else
static constexpr int patchSize_ = 6; // Edge length of the patches (in pixel). Must be a multiple of 2!
#endif

#ifdef RADVIO_NCAM
static constexpr int nCam_ = RADVIO_NCAM;
#else
static constexpr int nCam_ = 1; // Used total number of cameras.
#endif

#ifdef RADVIO_NPOSE
static constexpr int nPose_ = RADVIO_NPOSE;
#else
static constexpr int nPose_ = 0; // Additional pose states.
#endif

typedef radvio::RadvioFilter<radvio::FilterState<nMax_,nLevels_,patchSize_,nCam_,nPose_>> mtFilter;

#ifdef MAKE_SCENE
static radvio::RadvioScene<mtFilter> mRadvioScene;

void idleFunc(){
  ros::spinOnce();
  mRadvioScene.drawScene(mRadvioScene.mpFilter_->safe_);
}
#endif

int main(int argc, char** argv){
  ros::init(argc, argv, "radvio");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  std::string rootdir = ros::package::getPath("radvio"); // Leaks memory
  std::string filter_config = rootdir + "/cfg/radvio.info";

  nh_private.param("filter_config", filter_config, filter_config);

  // Filter
  std::shared_ptr<mtFilter> mpFilter(new mtFilter);
  mpFilter->readFromInfo(filter_config);

  // Force the camera calibration paths to the ones from ROS parameters.
  for (unsigned int camID = 0; camID < nCam_; ++camID) {
    std::string camera_config;
    if (nh_private.getParam("camera" + std::to_string(camID)
                            + "_config", camera_config)) {
      mpFilter->cameraCalibrationFile_[camID] = camera_config;
    }
    // Load per-camera image mask paths from ROS parameters
    std::string camera_mask;
    if (nh_private.getParam("camera" + std::to_string(camID) + "_mask", camera_mask)) {
      mpFilter->cameraImageMaskFile_[camID] = camera_mask;
    }
  }
  mpFilter->refreshProperties();

  // Node
  radvio::RadvioNode<mtFilter> radvioNode(nh, nh_private, mpFilter);
  radvioNode.makeTest();

#ifdef MAKE_SCENE
  // Scene
  std::string mVSFileName = rootdir + "/shaders/shader.vs";
  std::string mFSFileName = rootdir + "/shaders/shader.fs";
  mRadvioScene.initScene(argc,argv,mVSFileName,mFSFileName,mpFilter);
  mRadvioScene.setIdleFunction(idleFunc);
  mRadvioScene.addKeyboardCB('r',[&radvioNode]() mutable {radvioNode.requestReset();});
  glutMainLoop();
#else
  ros::spin();
#endif
  return 0;
}
