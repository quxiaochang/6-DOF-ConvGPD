#pragma once

#ifndef GRASP_H_
#define GRASP_H_

#include "pc_collect.h"

class GraspSet {
public:
	GraspSet(const UserPointType& sample_point_, const UserPointType& centroid_point_);
	GraspSet() = default;

	bool CalcuAxisXGrasp(Eigen::Vector3f& normal_vector);
	bool CalcuMatAxisYZ(Eigen::Vector3f& princom_vector);
	void CalcuTranMatrix();
	bool Algorithm1(Eigen::Vector3f& normal_vector, Eigen::Vector3f& princom_vector);
	bool Algorithm2(const Eigen::Matrix3f& curvature_axis);
	void FiltFixturePC(const pcl::PointCloud<UserPointType>& cloud_filtered, pcl::PointCloud<UserPointType>& cloud_fixture, size_t& pc_arm);
	bool OptimizeGrasp(const pcl::PointCloud<UserPointType>& cloud_filtered, pcl::PointCloud<UserPointType>& cloud_fixture, bool rotate_flag, ProcessCloud& pc);
	void OptimizeGraspDisplace(ProcessCloud& pc);
	void OptimizeGraspRotate(const pcl::PointCloud<UserPointType>& cloud_fixture, ProcessCloud& pc);
	bool OptimizeTest(const bool& optimize_test, Eigen::Vector3f& principal_vector);
	bool ClosureCollisionDetect(const pcl::PointCloud<UserPointType>& cloud_fixture);
	bool ArmCollisionDetect(const size_t& cloud_arm);
	bool PCNumberDetect(const pcl::PointCloud<UserPointType>& cloud_fixture);
	bool PostureDetect();
	bool PreScreenCand(const pcl::PointCloud<UserPointType>& cloud_fixture, const size_t& cloud_arm);
	void PlotNorm(const pcl::PointCloud<UserPointType>& cloud_filtered);
	void Plot3DGrasp(const pcl::PointCloud<UserPointType>& cloud_filtered, const pcl::PointCloud<UserPointType>& cloud_fixture);
	void Plot6DGrasp(const pcl::PointCloud<UserPointType>& cloud_filtered, const pcl::PointCloud<UserPointType>& cloud_fixture);

	/*以下变量都是以相机坐标系为参考*/
	UserPointType sample_point;   //表面采样点,亦是夹具坐标系的原点
	UserPointType grasp_point;    //抓取点，以实夹爪底部中心点
	UserPointType centroid_point; //质心点
	bool force_closure;           //抓取位姿是否满足力封闭约束
	float displace_x;             //grasp_point相当于sample_point沿X轴的偏移
	Eigen::Vector3f fix_axis_x;   //夹具坐标系向量
	Eigen::Vector3f fix_axis_y;
	Eigen::Vector3f fix_axis_z;  
	Eigen::Matrix4f fix2cam_mat; //夹具坐标系到相机坐标系转换矩阵
	//Eigen::Matrix4f ofix2fix_mat; //优化之后的夹具坐标系到之前夹具坐标系转换矩阵
};

typedef std::vector<GraspSet> GraspSets;

#endif

