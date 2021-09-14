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

	/*���±����������������ϵΪ�ο�*/
	UserPointType sample_point;   //���������,���Ǽо�����ϵ��ԭ��
	UserPointType grasp_point;    //ץȡ�㣬��ʵ��צ�ײ����ĵ�
	UserPointType centroid_point; //���ĵ�
	bool force_closure;           //ץȡλ���Ƿ����������Լ��
	float displace_x;             //grasp_point�൱��sample_point��X���ƫ��
	Eigen::Vector3f fix_axis_x;   //�о�����ϵ����
	Eigen::Vector3f fix_axis_y;
	Eigen::Vector3f fix_axis_z;  
	Eigen::Matrix4f fix2cam_mat; //�о�����ϵ���������ϵת������
	//Eigen::Matrix4f ofix2fix_mat; //�Ż�֮��ļо�����ϵ��֮ǰ�о�����ϵת������
};

typedef std::vector<GraspSet> GraspSets;

#endif

