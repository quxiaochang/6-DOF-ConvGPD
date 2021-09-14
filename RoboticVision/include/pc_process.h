#pragma once
/************************************************************************/
/* 功能：点云处理、变换、PCA、法线、框架                           */
/* 作者：瞿孝昌                                                    */
/* 版本：v1.0                                                      */
/************************************************************************/

#ifndef PC_PROCESS_H_
#define PC_PROCESS_H_

//system
#include <iostream>
#include <string>
#include <Eigen/Core>
#include <conio.h>
#include <algorithm>
#include <omp.h>
#include <chrono>
#include <math.h>
#include <boost/thread/thread.hpp>

//PCL
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/progressive_morphological_filter.h>
#include <pcl/common/transforms.h>
#include <pcl/common/common.h>
#include <pcl/common/common_headers.h>
#include <pcl/common/geometry.h>
#include <pcl/console/parse.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/random_sample.h>
#include <pcl/registration/transformation_estimation_svd.h>
#include <pcl/features/boundary.h>
#include <pcl/features/normal_3d_omp.h>

//USER
#include "config_file.h"

#define Pointnet 0
#define CovPoint 1
typedef pcl::PointXYZ UserPointType;

class ProcessCloud {
public:
	ProcessCloud(unsigned int led_value_, unsigned int working_mode_) :led_value(led_value_), working_mode(working_mode_) {}
	ProcessCloud() = default;

	void ProcessPC(const pcl::PointCloud<UserPointType>& cloud, pcl::PointCloud<UserPointType>& cloud_filtered) const;
	void SegmentPC(const pcl::PointCloud<UserPointType>& cloud_filtered, std::vector<pcl::PointCloud<UserPointType>>& output) const;
	size_t SegmentPC(const pcl::PointCloud<UserPointType>& cloud_filtered) const;
	bool VisualizePC(const pcl::PointCloud<UserPointType>& cloud_in, bool flag) const;
	void RoughPCA(const pcl::PointCloud<UserPointType>& cloud_cluster, const UserPointType& sample_point, std::vector<float>& manipulator_posture) const;
	bool RoughPCA(const pcl::PointCloud<UserPointType>& cloud_cluster, Eigen::Matrix3f& eigenVectorsPCA, float& value_ratio) const; //返回特征向量，从小到大
	bool ExtractBoundary(const pcl::PointCloud<UserPointType>& cloud_project, pcl::PointCloud<UserPointType>& cloud_boundary) const;
	bool CalNormalsOMP(const pcl::PointCloud<UserPointType>& cloud_in, const pcl::search::KdTree<UserPointType>::Ptr &tree, 
		pcl::PointCloud<pcl::Normal>& normals) const;
	bool CalCurvatureAxis(const pcl::PointCloud<UserPointType>& cloud_in, Eigen::Matrix3f& curvature_axis) const;  //返回特征向量，从小到大
	bool RadiusSearch(const float& search_radius, const int& min_pc, const UserPointType& searchPoint, 
		const pcl::PointCloud<UserPointType>& cloud_filtered, pcl::PointCloud<UserPointType>& cloud_cluster) const;
	void LineFit(const pcl::PointCloud<UserPointType>& cloud_in, pcl::ModelCoefficients::Ptr& coefficients) const;
	bool SampleVoxel(const pcl::PointCloud<UserPointType>& cloud_filtered, pcl::PointCloud<UserPointType>& cloud_samples) const;
	void ReadDataSet(pcl::PointCloud<UserPointType>& cloud_filtered);
	bool CalAngleY(const pcl::PointCloud<UserPointType>& cloud_fixture, float& theta_left, float& theta_right);

private:
	unsigned int led_value;
	unsigned int working_mode;

};

#endif



