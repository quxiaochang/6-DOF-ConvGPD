#pragma once

/************************************************************************/
/* 功能：卷积神经网络                                            */
/* 作者：瞿孝昌                                                    */
/* 版本：v1.0                                                      */
/************************************************************************/


#ifndef CNN_H__
#define CNN_H__

/*user*/
#include "grasp.h"
#include "cpp_py.h"
#include "arm.h"
#include "serial.h"

/*libtorch*/
//#include <torch/torch.h>
//#include <torch/script.h>



class CNN:public Python{
public:
	CNN(const int &cnn_name, const std::string &model_path);
	CNN() = default;

	bool NormFixturePC(const pcl::PointCloud<UserPointType>& cloud_fixture, pcl::PointCloud<UserPointType>& cloud_normalize);
	bool CollectTrainSet(const pcl::PointCloud<UserPointType>& cloud_fixture, pcl::PointCloud<UserPointType>& cloud_train);
	bool DatasetPc(const pcl::PointCloud<UserPointType>& cloud_fixture, float array[750][3]);
	
	//bool PC2Tensor(const pcl::PointCloud<UserPointType> &cloud_filtered, const std::vector<float> &mani_pos, torch::Tensor &tens_pc);
	//void Predict(const torch::Tensor &tens_pc, at::Tensor &output);
	bool GraspEmulate(const pcl::PointCloud<UserPointType>& cloud_filtered, std::vector<float>& best_grasp);
	bool PushEmulate(const pcl::PointCloud<UserPointType>& cloud_filtered, UserPointType& start_point, UserPointType& end_point);
	bool Run(SCAN& scanner, Conveyor& conveyor);
	void VisualizeGrasp(const pcl::PointCloud<UserPointType>& cloud_filtered, const GraspSets& grasp_sets, const uint16_t& best_index);

private:
	ProcessCloud pc;  //(30, 0)
	int cnn_model;   
	//torch::jit::script::Module module;

};

#endif
