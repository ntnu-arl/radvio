/*
* For parts of code or files added at the Autonomous Robots Lab, Norwegian University of Science and Technology, the
* following license is applicable:
*
* Copyright (c) 2026, Autonomous Robots Lab, Norwegian University of Science and Technology
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* 1. Redistributions of source code must retain the above copyright notice, this
*    list of conditions and the following disclaimer.
*
* 2. Redistributions in binary form must reproduce the above copyright notice,
*    this list of conditions and the following disclaimer in the documentation
*    and/or other materials provided with the distribution.
*
* 3. Neither the name of the copyright holder nor the names of its
*    contributors may be used to endorse or promote products derived from
*    this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
* FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
* DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
* For parts of code or files directly adopted from the Autonomous Systems Lab, following license is applicable (as from
* original repository for ROVIO: https://github.com/ethz-asl/rovio):
*
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

#include <ros/package.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <rosgraph_msgs/Clock.h>
#include <memory>
#include <iostream>
#include <locale>
#include <string>
#include <Eigen/StdVector>
#include "radvio/RadvioFilter.hpp"
#include "radvio/RadvioNode.hpp"
#include <boost/foreach.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/date_time/posix_time/posix_time_io.hpp>
#include <cstdio>
#define foreach BOOST_FOREACH

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
static constexpr int patchSize_ = 8; // Edge length of the patches (in pixel). Must be a multiple of 2!
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

int main(int argc, char** argv){
  ros::init(argc, argv, "radvio");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  ros::Publisher pubClock = nh_private.advertise<rosgraph_msgs::Clock>("/clock", 10);
  ros::param::set("/use_sim_time", true);

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
  double resetTrigger = 0.0;
  nh_private.param("record_odometry", radvioNode.forceOdometryPublishing_, radvioNode.forceOdometryPublishing_);
  nh_private.param("record_pose_with_covariance_stamped", radvioNode.forcePoseWithCovariancePublishing_, radvioNode.forcePoseWithCovariancePublishing_);
  nh_private.param("record_transform", radvioNode.forceTransformPublishing_, radvioNode.forceTransformPublishing_);
  nh_private.param("record_extrinsics", radvioNode.forceExtrinsicsPublishing_, radvioNode.forceExtrinsicsPublishing_);
  nh_private.param("record_imu_bias", radvioNode.forceImuBiasPublishing_, radvioNode.forceImuBiasPublishing_);
  nh_private.param("record_pcl", radvioNode.forcePclPublishing_, radvioNode.forcePclPublishing_);
  nh_private.param("record_markers", radvioNode.forceMarkersPublishing_, radvioNode.forceMarkersPublishing_);
  nh_private.param("record_patch", radvioNode.forcePatchPublishing_, radvioNode.forcePatchPublishing_);
  nh_private.param("record_targets_filtered", radvioNode.forceTargetsFilteredPublishing_, radvioNode.forceTargetsFilteredPublishing_);
  nh_private.param("record_targets_inlier", radvioNode.forceTargetsInlierPublishing_, radvioNode.forceTargetsInlierPublishing_);
  nh_private.param("record_depth_init", radvioNode.forceDepthInitPublishing_, radvioNode.forceDepthInitPublishing_);
  nh_private.param("reset_trigger", resetTrigger, resetTrigger);

  std::cout << "Recording";
  if(radvioNode.forceOdometryPublishing_) std::cout << ", odometry";
  if(radvioNode.forceTransformPublishing_) std::cout << ", transform";
  if(radvioNode.forceExtrinsicsPublishing_) std::cout << ", extrinsics";
  if(radvioNode.forceImuBiasPublishing_) std::cout << ", imu biases";
  if(radvioNode.forcePclPublishing_) std::cout << ", point cloud";
  if(radvioNode.forceMarkersPublishing_) std::cout << ", markers";
  if(radvioNode.forcePatchPublishing_) std::cout << ", patch data";
  if(radvioNode.forceTargetsFilteredPublishing_) std::cout << ", targetsfiltered";
  if(radvioNode.forceTargetsInlierPublishing_) std::cout << ", targetsinlier";
  if(radvioNode.forceDepthInitPublishing_) std::cout << ", depthInit";
  std::cout << std::endl;

  rosbag::Bag bagIn;
  std::string rosbag_filename = "dataset.bag";
  nh_private.param("rosbag_filename", rosbag_filename, rosbag_filename);
  bagIn.open(rosbag_filename, rosbag::bagmode::Read);

  rosbag::Bag bagOut;
  std::size_t found = rosbag_filename.find_last_of("/");
  std::string file_path = rosbag_filename.substr(0,found);
  std::string file_name = rosbag_filename.substr(found+1);
  if(file_path==rosbag_filename){
    file_path = ".";
    file_name = rosbag_filename;
  }

  std::stringstream stream;
  boost::posix_time::time_facet* facet = new boost::posix_time::time_facet();
  facet->format("%Y-%m-%d-%H-%M-%S");
  stream.imbue(std::locale(std::locale::classic(), facet));
  stream << ros::Time::now().toBoost() << "_" << nMax_ << "_" << nLevels_ << "_" << patchSize_ << "_" << nCam_  << "_" << nPose_;
  std::string filename_out = file_path + "/radvio/" + stream.str();
  nh_private.param("filename_out", filename_out, filename_out);
  std::string rosbag_filename_out = filename_out + ".bag";
  std::string info_filename_out = filename_out + ".info";
  std::string csv_filename_out = filename_out + ".csv";
  std::cout << "Storing output to: " << rosbag_filename_out << std::endl;
  bagOut.open(rosbag_filename_out, rosbag::bagmode::Write);

  FILE* odometryCsv = std::fopen(csv_filename_out.c_str(), "w");
  if (!odometryCsv)
  {
    // Handle error, e.g., print an error message and exit
    std::cerr << "Error opening file!" << std::endl;
    return 1;  // Indicate an error
  }
  std::fprintf(odometryCsv, "timestamp,x,y,z,qw,qx,qy,qz,vx,vy,vz\n");

  // Copy info
  std::ifstream  src(filter_config, std::ios::binary);
  std::ofstream  dst(info_filename_out,   std::ios::binary);
  dst << src.rdbuf();

  std::vector<std::string> topics;
  std::string imu_topic_name = "/imu0";
  nh_private.param("imu_topic_name", imu_topic_name, imu_topic_name);
  std::string cam0_topic_name = "/cam0/image_raw";
  nh_private.param("cam0_topic_name", cam0_topic_name, cam0_topic_name);
  std::string cam1_topic_name = "/cam1/image_raw";
  nh_private.param("cam1_topic_name", cam1_topic_name, cam1_topic_name);
  std::string radar_topic_name = "/radar/cloud";
  nh_private.param("radar_topic_name", radar_topic_name, radar_topic_name);
  std::string odometry_topic_name = radvioNode.pubOdometry_.getTopic();
  std::string transform_topic_name = radvioNode.pubTransform_.getTopic();
  std::string extrinsics_topic_name[mtFilter::mtState::nCam_];
  for(int camID=0;camID<mtFilter::mtState::nCam_;camID++){
    extrinsics_topic_name[camID] = radvioNode.pubExtrinsics_[camID].getTopic();
  }
  std::string radar_extrinsics_topic_name = radvioNode.pubRadarExtrinsics_.getTopic();
  std::string imu_bias_topic_name = radvioNode.pubImuBias_.getTopic();
  std::string pcl_topic_name = radvioNode.pubPcl_.getTopic();
  std::string u_rays_topic_name = radvioNode.pubMarkers_.getTopic();
  std::string patch_topic_name = radvioNode.pubPatch_.getTopic();
  std::string targets_filtered_topic_name = radvioNode.pubTargetsFiltered_.getTopic();
  std::string targets_inlier_topic_name = radvioNode.pubTargetsInliers_.getTopic();
  std::string candidate_voxels_topic_name = radvioNode.pubCandidateVoxels_.getTopic();
  std::string selected_voxels_topic_name = radvioNode.pubSelectedVoxels_.getTopic();

  topics.push_back(std::string(imu_topic_name));
  topics.push_back(std::string(cam0_topic_name));
  topics.push_back(std::string(cam1_topic_name));
  topics.push_back(std::string(radar_topic_name));
  rosbag::View view(bagIn, rosbag::TopicQuery(topics));


  bool isTriggerInitialized = false;
  double lastTriggerTime = 0.0;
  for(rosbag::View::iterator it = view.begin();it != view.end() && ros::ok();it++){
    rosgraph_msgs::Clock clock_msg;
    clock_msg.clock = it->getTime();
    pubClock.publish(clock_msg);

    if(it->getTopic() == imu_topic_name){
      sensor_msgs::Imu::ConstPtr imuMsg = it->instantiate<sensor_msgs::Imu>();
      if (imuMsg != NULL) radvioNode.imuCallback(imuMsg);
    }
    if(it->getTopic() == cam0_topic_name){
      sensor_msgs::ImageConstPtr imgMsg = it->instantiate<sensor_msgs::Image>();
      if (imgMsg != NULL) radvioNode.imgCallback0(imgMsg);
    }
    if(it->getTopic() == cam1_topic_name){
      sensor_msgs::ImageConstPtr imgMsg = it->instantiate<sensor_msgs::Image>();
      if (imgMsg != NULL) radvioNode.imgCallback1(imgMsg);
    }
    if (it->getTopic() == radar_topic_name){
      sensor_msgs::PointCloud2ConstPtr cloudMsg = it->instantiate<sensor_msgs::PointCloud2>();
      if (cloudMsg != NULL) radvioNode.radarCallback(cloudMsg);
    }
    ros::spinOnce();

    if(radvioNode.gotFirstMessages_){
      static double lastSafeTime = radvioNode.mpFilter_->safe_.t_;
      if(radvioNode.mpFilter_->safe_.t_ > lastSafeTime){
        // write odometry to csv
        const double ts = radvioNode.odometryMsg_.header.stamp.toSec();
        // position
        const double x = radvioNode.odometryMsg_.pose.pose.position.x;
        const double y = radvioNode.odometryMsg_.pose.pose.position.y;
        const double z = radvioNode.odometryMsg_.pose.pose.position.z;
        // attitude
        const double qw = radvioNode.odometryMsg_.pose.pose.orientation.w;
        const double qx = radvioNode.odometryMsg_.pose.pose.orientation.x;
        const double qy = radvioNode.odometryMsg_.pose.pose.orientation.y;
        const double qz = radvioNode.odometryMsg_.pose.pose.orientation.z;
        // velocity
        const double vx = radvioNode.odometryMsg_.twist.twist.linear.x;
        const double vy = radvioNode.odometryMsg_.twist.twist.linear.y;
        const double vz = radvioNode.odometryMsg_.twist.twist.linear.z;
        // writing to csv
        std::fprintf(odometryCsv, "%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f\n", ts, x, y, z, qw, qx, qy, qz, vx, vy, vz);

        if(radvioNode.forceOdometryPublishing_) bagOut.write(odometry_topic_name,ros::Time::now(),radvioNode.odometryMsg_);
        if(radvioNode.forceTransformPublishing_) bagOut.write(transform_topic_name,ros::Time::now(),radvioNode.transformMsg_);
        for(int camID=0;camID<mtFilter::mtState::nCam_;camID++){
          if(radvioNode.forceExtrinsicsPublishing_) bagOut.write(extrinsics_topic_name[camID],ros::Time::now(),radvioNode.extrinsicsMsg_[camID]);
        }
        if(radvioNode.forceExtrinsicsPublishing_) bagOut.write(radar_extrinsics_topic_name,ros::Time::now(),radvioNode.radarExtrinsicsMsg_);
        if(radvioNode.forceImuBiasPublishing_) bagOut.write(imu_bias_topic_name,ros::Time::now(),radvioNode.imuBiasMsg_);
        if(radvioNode.forcePclPublishing_) bagOut.write(pcl_topic_name,ros::Time::now(),radvioNode.pclMsg_);
        if(radvioNode.forceMarkersPublishing_) bagOut.write(u_rays_topic_name,ros::Time::now(),radvioNode.markerMsg_);
        if(radvioNode.forcePatchPublishing_) bagOut.write(patch_topic_name,ros::Time::now(),radvioNode.patchMsg_);
        if (radvioNode.forceTargetsFilteredPublishing_ && radvioNode.targetsFilteredMsg_.header.stamp.toSec() > 0)
        {
          bagOut.write(targets_filtered_topic_name, ros::Time::now(), radvioNode.targetsFilteredMsg_);
          radvioNode.targetsFilteredMsg_.header.stamp = ros::Time(0);
        }
        if (radvioNode.forceTargetsInlierPublishing_ && radvioNode.targetsInlierMsg_.header.stamp.toSec() > 0)
        {
          bagOut.write(targets_inlier_topic_name, ros::Time::now(), radvioNode.targetsInlierMsg_);
          radvioNode.targetsInlierMsg_.header.stamp = ros::Time(0);
        }
        if (radvioNode.forceDepthInitPublishing_)
        {
          bagOut.write(candidate_voxels_topic_name, ros::Time::now(), radvioNode.candidateVoxelMsg_);
          bagOut.write(selected_voxels_topic_name, ros::Time::now(), radvioNode.selectedVoxelMsg_);
        }
        lastSafeTime = radvioNode.mpFilter_->safe_.t_;
      }
      if(!isTriggerInitialized){
        lastTriggerTime = lastSafeTime;
        isTriggerInitialized = true;
      }
      if(resetTrigger>0.0 && lastSafeTime - lastTriggerTime > resetTrigger){
        radvioNode.requestReset();
        radvioNode.mpFilter_->init_.state_.WrWM() = radvioNode.mpFilter_->safe_.state_.WrWM();
        radvioNode.mpFilter_->init_.state_.qWM() = radvioNode.mpFilter_->safe_.state_.qWM();
        lastTriggerTime = lastSafeTime;
      }
    }
  }

  bagOut.close();
  bagIn.close();
  std::fclose(odometryCsv);


  return 0;
}
