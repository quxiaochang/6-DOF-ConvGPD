#pragma warning(disable:4996)
#include "cnn.h"

CNN::CNN(const int &cnn_name, const std::string &model_path):pc(30, 0) {
	cnn_model = cnn_name;
	if (cnn_name == CovPoint) {
		/*CovPoint*/
		ModuleInit();
	}
	else {
		/*PointNet Init model*/
		std::cout << "Start to load model" << std::endl;
		//module = torch::jit::load(model_path);
		//module.to(at::kCPU);
		std::cout << "Success to load model" << std::endl;
	}
}

//bool CNN::FiltFixturePC(const pcl::PointCloud<UserPointType>& cloud_filtered, GraspSet& graspset, pcl::PointCloud<UserPointType>& cloud_fixture) {
//	cloud_fixture.clear();
//
//	pcl::PointCloud<UserPointType>::Ptr cloud_transform(new pcl::PointCloud<UserPointType>); //ת�����о�����ϵ														
//	pcl::transformPointCloud(cloud_filtered, *cloud_transform, graspset.fix2cam_mat); //cloud_transform = mat* cloud_filtered
//
//	pcl::PointCloud<UserPointType>::Ptr cloud_arm(new pcl::PointCloud<UserPointType>);
//	pcl::PassThrough<UserPointType> pass;
//	/*ת��֮��ֱͨ�˲�*/
//	pass.setInputCloud(cloud_transform);
//	pass.setFilterFieldName("z");
//	pass.setFilterLimits(-param::finger_z, param::finger_z);  //��λ��mm
//	pass.filter(cloud_fixture);
//
//	pass.setInputCloud(cloud_fixture.makeShared());
//	pass.setFilterFieldName("y");
//	pass.setFilterLimits(-param::finger_y, param::finger_y);
//	pass.filter(cloud_fixture);
//
//	pass.setInputCloud(cloud_fixture.makeShared());
//	pass.setFilterFieldName("x");
//	pass.setFilterLimits(graspset.displace_x - 100, graspset.displace_x);
//	pass.filter(*cloud_arm);
//
//	pass.setInputCloud(cloud_fixture.makeShared());
//	pass.setFilterFieldName("x");
//	pass.setFilterLimits(graspset.displace_x, (2 * param::finger_x + graspset.displace_x));
//	pass.filter(cloud_fixture);
//
//	std::cout << "Points in the fixture : " << cloud_fixture.size() << std::endl;
//
//	if (param::plot_pc) {
//		graspset.Plot3DGrasp(cloud_filtered, cloud_fixture);
//		graspset.Plot6DGrasp(*cloud_transform, cloud_fixture);
//	}
//
//	int min_pc_fix(0);
//	if (param::dataset_test)  //YCB�����ݼ��������ϡ�裬���趨Сһ��
//		min_pc_fix = 10;
//	else
//		min_pc_fix = 50;
//
//	bool flag = cloud_fixture.size() > min_pc_fix && cloud_arm->size() < 10;  //100�޸�Ϊ20
//	if (flag == false)
//		std::cout << "Too few pc in fixture or a collection with arm!" << std::endl;
//
//	return flag;//ע�⣬����cloud_fixtur������С(<50),��Ҫ����
//}

bool CNN::NormFixturePC(const pcl::PointCloud<UserPointType>& cloud_fixture, pcl::PointCloud<UserPointType>& cloud_normalize) {
	cloud_normalize.clear();

	for (auto point : cloud_fixture.points) {     //�о�����ϵ�µ��ƹ�һ��
		cloud_normalize.points.emplace_back(UserPointType(point.x / param::finger_max, 
			point.y / param::finger_max, point.z / param::finger_max)); //��һ������
	}
	return cloud_normalize.size() > 0;
}

bool CNN::CollectTrainSet(const pcl::PointCloud<UserPointType> &cloud_fixture, pcl::PointCloud<UserPointType> &cloud_train) {
	cloud_train.clear();

	NormFixturePC(cloud_fixture, cloud_train); //�о�����ϵ���ƹ�һ������

	/*���ȸ����������,ʹ��ĸ�����Ϊ750*/
	pcl::RandomSample<UserPointType> rs;
	rs.setInputCloud(cloud_train.makeShared());
	if (cloud_train.size() < param::set_sample) {
		do {
			pcl::PointCloud<UserPointType>::Ptr cloud_temp(new pcl::PointCloud<UserPointType>);
			rs.setSample((param::set_sample - cloud_train.size()));//�������������� 
			rs.filter(*cloud_temp);
			cloud_train = cloud_train + *cloud_temp;  //�ϲ�����
			std::cout << "The number of points in fixture is not enough!" << std::endl;
		} while (cloud_train.size() < param::set_sample);
	}
	else {
		rs.setSample(param::set_sample);//�������������� 
		rs.filter(cloud_train);   //cloud_trainΪ�о�����ϵ,�о���,��һ��,750�ĵ���
	}
	std::cout << "Points in the fixture after random sampling: " << cloud_train.size() << std::endl;

	if (param::collect_train) {
		pcl::io::savePCDFileASCII("D:\\Doraemon\\pcd_qxc\\dataset_qxc\\0.pcd", cloud_train);
		std::cout << "Train dataset has been saved!" << std::endl;
	}
	return cloud_train.size() == 750;
}


//bool PointNet::PC2Tensor(const pcl::PointCloud<UserPointType> &cloud_filtered, 
//const std::vector<float> &mani_pos, torch::Tensor &tens_pc) {
//�о�����ϵ��һ��ѵ����
//	pcl::PointCloud<UserPointType>::Ptr cloud_train(new pcl::PointCloud<PointType>); 
//cloud_filteredת�����о�����ϵ,��һ��,ֱͨ�˲�,���Ȳ���750
//	if (!CollectTrainSet(cloud_filtered, mani_pos, *cloud_train))
//		return 0;  //ע�⣬����cloud_train������С(<50),��Ҫ����
//
//	/*���Ļ�*/
//	Eigen::Vector4f pcaCentroid;
//	pcl::compute3DCentroid(*cloud_train, pcaCentroid);
//	std::cout << "�о���PC������: " << pcaCentroid << std::endl;
//
//
//	std::vector<float> scales;
//	for (auto point : cloud_train->points) {
//		scales.emplace_back(point.x - pcaCentroid[0]);
//		scales.emplace_back(point.y - pcaCentroid[1]);
//		scales.emplace_back(point.z - pcaCentroid[2]);
//	}
//	std::cout << "���Ļ�vector: " << scales.size() << std::endl;
//
//	tens_pc = torch::tensor(scales); //torch.Size([750*3])
//	tens_pc = tens_pc.reshape({ -1,3 }).unsqueeze(0).transpose(2, 1);  //[750*3]->[750,3]->[1,750,3]->[1,3,750]
//	std::cout << "tens_pc.sizes: " << tens_pc.sizes() << std::endl;
//	return scales.size() > 0;
//}
//
//void PointNet::Predict(const torch::Tensor &tens_pc, at::Tensor &output) {
//	std::vector<torch::jit::IValue> inputs;
//	inputs.push_back(tens_pc);
//
//	// Execute the model and turn its output into a tensor.
//	output = module.forward(inputs).toTensor();
//	std::tuple<torch::Tensor, torch::Tensor> max_test = torch::max(output, 1); //���������ֵ
//	auto max_val = std::get<0>(max_test);	// value
//	auto index = std::get<1>(max_test);	    // index
//	std::cout << "Success to Predict" << std::endl;
//	std::cout << "output: " << output << "   " << output[0][2].item().toFloat() << std::endl;
//	std::cout << "output.max: " << max_val.item().toFloat() << " " << index.item().toLong() << std::endl;
//	if (index.item().toLong() == 2)
//		std::cout << "Grasp is best!" << std::endl;
//}

bool CNN::DatasetPc(const pcl::PointCloud<UserPointType> &cloud_fixture, float array[750][3]) {
	pcl::PointCloud<UserPointType>::Ptr cloud_train(new pcl::PointCloud<UserPointType>); //�о�����ϵ��һ��ѵ����
	/*cloud_filteredת�����о�����ϵ,ֱͨ�˲�,��һ��,���Ȳ���750*/
	if (!CollectTrainSet(cloud_fixture, *cloud_train))
		return 0;  //ע�⣬����cloud_train������С(<50),��Ҫ����

	/*���Ļ�*/
	Eigen::Vector4f pcaCentroid;
	pcl::compute3DCentroid(*cloud_train, pcaCentroid);
	std::cout << "�о���PC������: " << pcaCentroid << std::endl;

	int i = 0;
	for (auto point : cloud_train->points) {
		array[i][0] = (point.x - pcaCentroid[0]);
		array[i][1] = (point.y - pcaCentroid[1]);
		array[i][2] = (point.z - pcaCentroid[2]);
		i++;
	}
	std::cout << "��һ�����Ļ�array: " << i << std::endl;
	return i == 750;
}

bool CNN::GraspEmulate(const pcl::PointCloud<UserPointType> &cloud_filtered, std::vector<float> &best_grasp) {
	best_grasp.clear();
	std::cout << "Start to emulate grasp" << std::endl;
	pcl::PointCloud<UserPointType>::Ptr cloud_samples(new pcl::PointCloud<UserPointType>);  //��������
	pcl::PointCloud<UserPointType>::Ptr cloud_cluster1(new pcl::PointCloud<UserPointType>);  //������������
	pcl::PointCloud<UserPointType>::Ptr cloud_cluster2(new pcl::PointCloud<UserPointType>);
	pcl::PointCloud<UserPointType>::Ptr cloud_fixture(new pcl::PointCloud<UserPointType>); //�о�����ϵ����

	/*���ز���*/
	pc.SampleVoxel(cloud_filtered, *cloud_samples);

	/*���ļ���*/
	Eigen::Vector4f centroid;
	pcl::compute3DCentroid(cloud_filtered, centroid);
	UserPointType centroid_point(centroid(0), centroid(1), centroid(2));

	int min_pc(0);
	if (param::dataset_test)  //YCB�����ݼ��������ϡ�裬���趨Сһ��
		min_pc = 10;
	else
		min_pc = 70;

	Arm arm(79, 154.8, 188, 85);
	GraspSets graspsets;
	std::vector<float> sorces;
	float value_ratio(0), ratio(0);
	size_t force_count(0);
	size_t pc_arm(0); //��е�������ڲ���������

	Eigen::Matrix3f eigenVectorsPCA1 = Eigen::Matrix3f::Identity();  //Э�����������������������ֵ��С��������
	Eigen::Matrix3f eigenVectorsPCA2 = Eigen::Matrix3f::Identity();

	for (auto searchPoint : cloud_samples->points) {
		std::cout << "searchPoint:" << searchPoint << std::endl;

		//UserPointType p(-23.7437, -17.0488, 389.602);//(-23.7437,-17.0488,389.602)
		//searchPoint = p; 
		GraspSet graspset(searchPoint, centroid_point);

		if (!pc.RadiusSearch(param::kdtree_rad, 3, searchPoint, cloud_filtered, *cloud_cluster1))//3��ȷ��ƽ��
			continue;
		pc.RoughPCA(*cloud_cluster1, eigenVectorsPCA1, value_ratio);
		Eigen::Vector3f normal_vector = eigenVectorsPCA1.col(0);

		if (!pc.RadiusSearch(param::finger_y - 10, min_pc, searchPoint, cloud_filtered, *cloud_cluster2))
			continue;
		pc.RoughPCA(*cloud_cluster2, eigenVectorsPCA2, ratio);
		Eigen::Vector3f principal_vector = eigenVectorsPCA2.col(2);

		std::cout << "Start to execute Algorithm1" << std::endl;
		graspset.Algorithm1(normal_vector, principal_vector);

		/*����һ֮���Ż�*/
		if (!graspset.OptimizeGrasp(cloud_filtered, *cloud_fixture, 0, pc)) {  
			std::cout << "A collision in Algorithm1 after optimization!" << std::endl;
			std::cout << "value_ratio:" << value_ratio << std::endl;

			if (1) { //����sample_point�ڱ�Ե value_ratio > 1.0
				std::cout << "Start to execute Algorithm2" << std::endl;

				//Eigen::Matrix3f curvature_axis = Eigen::Matrix3f::Identity();//���ʲ�����
				//pc.CalCurvatureAxis(*cloud_cluster2, curvature_axis);
				graspset.Algorithm2(eigenVectorsPCA2);  //curvature_axis

				/*������֮���Ż�*/
				if (!graspset.OptimizeGrasp(cloud_filtered, *cloud_fixture, 1, pc)) {
					std::cout << "A collision in Algorithm2 after optimization!" << std::endl;
					continue;
				}
			}
			else {
				std::cout << "Abandon this sample point!" << std::endl;
				continue;
			}
		}

		std::vector<float> predict;
		//if (cnn_model == CovPoint) {  //CovPointģʽ
		float array[750][3];
		if (!DatasetPc(*cloud_fixture, array))
			continue;

		std::cout << "Start to CovPoint predict" << std::endl;
		if (!Estimator(array, predict)) 
			continue;
		
		float final_sorce = predict[2];
		final_sorce += (param::hight_coe * (graspset.grasp_point.z) +
			param::y_coe * (graspset.grasp_point.y));                        //��Ӹ߶�Ȩ��
		std::cout << "In C++ predict sorce: " << final_sorce << std::endl;
		//}
		//else {  //PointNetģʽ
		//	torch::Tensor tens_pc;
		//	at::Tensor output;
		//	if (!PC2Tensor(*cloud_filtered, manipulator_posture, tens_pc))
		//		continue;
		//	Predict(tens_pc, output);

		//	std::tuple<torch::Tensor, torch::Tensor> max_test = torch::max(output, 1); //���������ֵ
		//	auto max_val = std::get<0>(max_test);	// value
		//	auto index = std::get<1>(max_test);	    // index

		//	float sorce = 0;
		//	if (param::multi_detect)
		//		sorce = output[0][2].item().toFloat() + ((param::weight_coe) * (420 - manipulator_posture[2]));
		//	else
		//		sorce = output[0][2].item().toFloat();
		//}

		/*���������Լ���ж�ץȡλ���Ƿ��ȶ�*/
		if (param::force_closure_test) {
			float theta_left(0), theta_right(0); //�����ƣ���Y��н�

			pc.CalAngleY(*cloud_fixture, theta_left, theta_right);

			float a1 = abs(theta_left - M_PI_2) / M_PI * 180; //�Ƕ��ƣ���Z��н�
			float a2 = abs(theta_right - M_PI_2) / M_PI * 180;
			std::cout << "a1 after optimization: " << a1 << std::endl;
			std::cout << "a2 after optimization:: " << a2 << std::endl;

			if (a1 < 16.7 && a2 < 16.7) {  //ȡĦ��ϵ��u=0.3����Ħ��Բ׶��Ϊ16.7��
				force_count++;
				graspset.force_closure = true;
			}
			else
				graspset.force_closure = false;
		}
		sorces.emplace_back(final_sorce);  //���յ÷�����final_sorce
		graspsets.emplace_back(graspset);
	}

	if (!graspsets.empty()) {
		if (param::force_closure_test) {
			float force_closure_rate = float(force_count) / float(graspsets.size());
			std::cout << "graspsets.size(): " << graspsets.size() << std::endl;
			std::cout << "force_count: " << force_count << std::endl;
			std::cout << "force_closure_rate: " << force_closure_rate << std::endl;
		}

		auto max_sorce = std::max_element(sorces.begin(), sorces.end());
		auto max_index = std::distance(sorces.begin(), max_sorce);

		GraspSet best_graspset(graspsets[max_index]);   //����ץȡλ��

		//arm.ManiInverse(best_graspset, best_grasp);
		best_grasp.emplace_back(1);

		std::cout << "max_index: " << max_index << std::endl;
		if (param::plot_grasp)
			VisualizeGrasp(cloud_filtered, graspsets, max_index);

		return !best_grasp.empty();
	}
	else {  //û�з�������ץȡλ�ˣ����������㷨
		std::cout << "The Grasp Sets are empty!" << std::endl;
		return false;
	}
}

bool CNN::PushEmulate(const pcl::PointCloud<UserPointType>& cloud_filtered, UserPointType& start_point, UserPointType& end_point) {
	std::cout << "Start to emulate push" << std::endl;
	pcl::PointCloud<UserPointType>::Ptr cloud_boundary(new pcl::PointCloud<UserPointType>);
	pcl::PointCloud<UserPointType>::Ptr cloud_cluster(new pcl::PointCloud<UserPointType>);
	
	if (!pc.ExtractBoundary(cloud_filtered, *cloud_boundary))
		return false;

	Eigen::Vector4f centroid_vec;  //����
	pcl::compute3DCentroid(*cloud_boundary, centroid_vec);

	UserPointType centroid(centroid_vec(0), centroid_vec(1), centroid_vec(2));
	std::vector <float> distances;

	for (auto point : cloud_boundary->points) {
		distances.emplace_back(pcl::geometry::distance(point, centroid));
	}
	auto min_distance = std::min_element(distances.begin(), distances.end());
	auto min_index = std::distance(distances.begin(), min_distance);

	start_point = cloud_boundary->points[min_index];  //��ʼ��start_poin

	if (!pc.RadiusSearch(param::kdtree_rad, 3, start_point, *cloud_boundary, *cloud_cluster))
		return false;

	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
	pc.LineFit(*cloud_cluster, coefficients);

	Eigen::Vector3f boundary_vector  = Eigen::Vector3f::Identity();  //����ֱ������
	Eigen::Vector3f gravity_vector = Eigen::Vector3f::Identity();  //������������
	boundary_vector << coefficients->values[3], coefficients->values[4], coefficients->values[5];
	gravity_vector << 0, 0, 1;

	Eigen::Vector3f push_vector = boundary_vector.cross(gravity_vector);  //�ƶ�����
	push_vector.normalize();

	float scl = 30; //�ƶ�����
	end_point.x = scl * push_vector(0) + start_point.x; //��ʼ��end_point
	end_point.y = scl * push_vector(1) + start_point.y;
	end_point.z = scl * push_vector(2) + start_point.z;

	if (param::plot_grasp) {
		boundary_vector.normalize();
		UserPointType pcA(25 * boundary_vector(0) + start_point.x, 25 * boundary_vector(1) + start_point.y,
			25 * boundary_vector(2) + start_point.z);
		UserPointType pcB(-25 * boundary_vector(0) + start_point.x, -25 * boundary_vector(1) + start_point.y,
			-25 * boundary_vector(2) + start_point.z);

		pcl::visualization::PCLVisualizer Viewer;
		pcl::visualization::PointCloudColorHandlerCustom<UserPointType> color_handler(cloud_filtered.makeShared(), 255, 255, 255);//0, 229, 238
		Viewer.addPointCloud(cloud_filtered.makeShared(), color_handler, "cloud_filtered");
		Viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud_filtered");
		Viewer.addSphere(start_point, 0.8, 255, 0, 0, "start_point");
		/*������Ʒ���*/
		Viewer.addArrow(end_point, start_point, 0.0, 1.0, 0.0, false, "push");
		/*������ֱ��*/
		Viewer.addLine(pcA, pcB, 0, 0, 255, "AB");
		//Viewer.setBackgroundColor(255, 255, 255);
		while (!Viewer.wasStopped())
		{
			Viewer.spinOnce(100);
		}
	}
	return true;
}

bool CNN::Run(SCAN& scanner, Conveyor& conveyor) { //best_grasp��Ҫ���ݻ�е���޸�����
	pcl::PointCloud<UserPointType>::Ptr cloud(new pcl::PointCloud<UserPointType>);
	pcl::PointCloud<UserPointType>::Ptr cloud_filtered(new pcl::PointCloud<UserPointType>);
	bool grasp_flag = false;
	UserPointType start_point, end_point;

	/*��ȡ��������*/
	if (param::dataset_test)
		pc.ReadDataSet(*cloud_filtered);
	else {
		if (param::camera_test) {
			conveyor.motor_on(); //�������ʹ�
			getchar();
			if (!scanner.ConnectCamera()) {//3Dͬ��ɨ��
				conveyor.motor_off();  //ֹͣ���ʹ�
				return false;
			}
			*cloud = scanner.get_cloud_original();
			conveyor.motor_off();  //ֹͣ���ʹ�
		}
		else
			pcl::io::loadPCDFile<UserPointType>(param::pcd_path, *cloud);

		if (param::plot_pc) {
			pc.VisualizePC(*cloud, 1);
			//pcl::io::savePCDFileASCII("D:\\Doraemon\\pcd_qxc\\LMI\\LMI_pc.pcd", *cloud);
			std::cout << "Success to save PCD, cloud_original.size: " << cloud->size() << std::endl;
		}
		pc.ProcessPC(*cloud, *cloud_filtered); //Ԥ����,�������ϵ,cloud_filtered�ǳ���Ҫ,�Ҳ����ٱ�����
	}
	auto start_time = std::chrono::steady_clock::now();

	std::vector<float> best_grasp;
	grasp_flag = GraspEmulate(*cloud_filtered, best_grasp);

	auto end_time = std::chrono::steady_clock::now();
	auto total_span = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
	std::cout << "GraspEmulate runtime:" << total_span.count() << " ms" << std::endl;
	getchar();

	if(grasp_flag == false && cloud_filtered->size() > 500) //�������岢��û���ҵ����ʵ�ץȡλ��
		PushEmulate(*cloud_filtered, start_point, end_point);

	return grasp_flag;
}

void CNN::VisualizeGrasp(const pcl::PointCloud<UserPointType>& cloud_filtered, const GraspSets& grasp_sets, const uint16_t& best_index) {
	
	pcl::visualization::PCLVisualizer Viewers;
	pcl::visualization::PointCloudColorHandlerCustom<UserPointType> color_handler(cloud_filtered.makeShared(), 0, 255, 255);  //0, 229, 238
	Viewers.addPointCloud(cloud_filtered.makeShared(), color_handler, "cloud_filtered");
	Viewers.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud_filtered");


	uint16_t i(0), j(0); //uint16_t i�����ʼ��
	for (auto grasp_set:grasp_sets) {
		//visualization
		UserPointType pcA, pcB, pcC, pcD, pcE, pcF, cp;
		cp = grasp_set.sample_point;
		pcA = grasp_set.grasp_point;

		pcB.x = (grasp_set.displace_x - 20) * grasp_set.fix_axis_x(0) + cp.x;
		pcB.y = (grasp_set.displace_x - 20) * grasp_set.fix_axis_x(1) + cp.y;
		pcB.z = (grasp_set.displace_x - 20) * grasp_set.fix_axis_x(2) + cp.z;

		pcC.x = -param::finger_y * grasp_set.fix_axis_y(0) + pcA.x;
		pcC.y = -param::finger_y * grasp_set.fix_axis_y(1) + pcA.y;
		pcC.z = -param::finger_y * grasp_set.fix_axis_y(2) + pcA.z;

		pcD.x = 2 * param::finger_x * grasp_set.fix_axis_x(0) + pcC.x;
		pcD.y = 2 * param::finger_x * grasp_set.fix_axis_x(1) + pcC.y;
		pcD.z = 2 * param::finger_x * grasp_set.fix_axis_x(2) + pcC.z;

		pcE.x = param::finger_y * grasp_set.fix_axis_y(0) + pcA.x;
		pcE.y = param::finger_y * grasp_set.fix_axis_y(1) + pcA.y;
		pcE.z = param::finger_y * grasp_set.fix_axis_y(2) + pcA.z;

		pcF.x = 2 * param::finger_x * grasp_set.fix_axis_x(0) + pcE.x;
		pcF.y = 2 * param::finger_x * grasp_set.fix_axis_x(1) + pcE.y;
		pcF.z = 2 * param::finger_x * grasp_set.fix_axis_x(2) + pcE.z;

		if (1) {  //i != best_index i == best_indexgrasp_set.force_closure == true
			Viewers.addLine(pcA, pcB, 1, 0.7569, 0.1451, std::to_string(j + 1e3)); //"AB"
			Viewers.addLine(pcC, pcE, 1, 0.7569, 0.1451, std::to_string(j + 2e3));
			Viewers.addLine(pcC, pcD, 1, 0.7569, 0.1451, std::to_string(j + 3e3));
			Viewers.addLine(pcE, pcF, 1, 0.7569, 0.1451, std::to_string(j + 4e3));
		}
		else {  //����ץȡλ��
			Viewers.addLine(pcA, pcB, 1, 0, 0, std::to_string(j + 1e3));
			Viewers.addLine(pcC, pcE, 1, 0, 0, std::to_string(j + 2e3));
			Viewers.addLine(pcC, pcD, 1, 0, 0, std::to_string(j + 3e3));
			Viewers.addLine(pcE, pcF, 1, 0, 0, std::to_string(j + 4e3));
			//std::cout << "FFF!!! sample point which is false:" << grasp_set.sample_point << std::endl;
			  //j++;
		}
		Viewers.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 5, std::to_string(j + 1e3));
		Viewers.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 5, std::to_string(j + 2e3));
		Viewers.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 5, std::to_string(j + 3e3));
		Viewers.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 5, std::to_string(j + 4e3));

		i++;
		j++;
	}
	Viewers.setBackgroundColor(255, 255, 255);
	while (!Viewers.wasStopped())
	{
		Viewers.spinOnce(100);
	}
}





