#pragma warning(disable:4996)
#include "pc_process.h"

void ProcessCloud::ProcessPC(const pcl::PointCloud<UserPointType>& cloud, pcl::PointCloud<UserPointType>& cloud_filtered) const {
	cloud_filtered.clear();
	auto start_time = std::chrono::steady_clock::now();

	/*�����˲�������*/
	pcl::PassThrough<UserPointType> pass;
	pass.setInputCloud(cloud.makeShared());
	pass.setFilterFieldName("z");
	pass.setFilterLimits(param::workspace[4], param::workspace[5]);  //��λ��mm,�������Ч����Ϊ340-440mm
	pass.filter(cloud_filtered);

	//pass.setInputCloud(cloud_filtered.makeShared());
	//pass.setFilterFieldName("y");
	//pass.setFilterLimits(param::workspace[2], param::workspace[3]);
	//pass.filter(cloud_filtered);

	//pass.setInputCloud(cloud_filtered.makeShared());
	//pass.setFilterFieldName("x");
	//pass.setFilterLimits(param::workspace[0], param::workspace[1]);
	//pass.filter(cloud_filtered);
	std::cout << "Cloud after filiter: " << cloud_filtered.size() << std::endl;

	/*�²���*/
	pcl::VoxelGrid<UserPointType> sor;
	sor.setInputCloud(cloud_filtered.makeShared());
	sor.setLeafSize(param::cluster_extract[3], param::cluster_extract[3], param::cluster_extract[3]);
	sor.filter(cloud_filtered);
	std::cout << "Cloud after subsample: " << cloud_filtered.size() << std::endl;
	if (param::plot_pc)
		VisualizePC(cloud_filtered, 1);
	/*��Ⱥ���Ƴ�*/
	pcl::StatisticalOutlierRemoval<UserPointType> sta;
	sta.setInputCloud(cloud_filtered.makeShared());
	sta.setMeanK(50);
	sta.setStddevMulThresh(1.5);  //ԽС���˵ĵ�Խ��1.3
	sta.filter(cloud_filtered);
	std::cout << "Cloud after removing statistical outliers: " << cloud_filtered.size() << std::endl;

	auto end_time = std::chrono::steady_clock::now();
	auto total_span = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
	std::cout << "Process PC runtime:" << total_span.count() << " ms" << std::endl;

	if (param::plot_pc)
		VisualizePC(cloud_filtered, 1);
}

void ProcessCloud::SegmentPC(const pcl::PointCloud<UserPointType>& cloud_filtered, std::vector<pcl::PointCloud<UserPointType>>& output) const {
	// Creating the KdTree object for the search method of the extraction
	output.clear();
	pcl::search::KdTree<UserPointType>::Ptr tree(new pcl::search::KdTree<UserPointType>);
	tree->setInputCloud(cloud_filtered.makeShared());

	std::vector<pcl::PointIndices> cluster_indices;
	pcl::EuclideanClusterExtraction<UserPointType> ec;
	ec.setClusterTolerance(param::cluster_extract[0]); // ȡ̫Сֵ�����ָ����࣬��֮�ָ�Ϊһ����
	ec.setMinClusterSize(param::cluster_extract[1]);
	ec.setMaxClusterSize(param::cluster_extract[2]);
	ec.setSearchMethod(tree);
	ec.setInputCloud(cloud_filtered.makeShared());
	ec.extract(cluster_indices);
	std::cout << "Cloud is divided into: " << cluster_indices.size() << " categories" << std::endl;

	for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
	{
		pcl::PointCloud<UserPointType>::Ptr cloud_cluster(new pcl::PointCloud<UserPointType>);
		for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); pit++)
			cloud_cluster->points.emplace_back(cloud_filtered.points[*pit]); //*
		cloud_cluster->width = cloud_cluster->points.size();
		cloud_cluster->height = 1;
		cloud_cluster->is_dense = true;
		output.emplace_back(*cloud_cluster);

		std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size() << " data points." << std::endl;
	}
}

size_t ProcessCloud::SegmentPC(const pcl::PointCloud<UserPointType>& cloud_filtered) const {
	// Creating the KdTree object for the search method of the extraction
	pcl::search::KdTree<UserPointType>::Ptr tree(new pcl::search::KdTree<UserPointType>);
	tree->setInputCloud(cloud_filtered.makeShared());

	std::vector<pcl::PointIndices> cluster_indices;
	pcl::EuclideanClusterExtraction<UserPointType> ec;
	ec.setClusterTolerance(6); // ȡ̫Сֵ�����ָ����࣬��֮�ָ�Ϊһ����
	ec.setMinClusterSize(1);
	ec.setMaxClusterSize(5000);
	ec.setSearchMethod(tree);
	ec.setInputCloud(cloud_filtered.makeShared());
	ec.extract(cluster_indices);
	std::cout << "Cloud is divided into: " << cluster_indices.size() << " categories" << std::endl;

	return cluster_indices.size();
}

bool ProcessCloud::VisualizePC(const pcl::PointCloud<UserPointType>& cloud_in, bool flag) const {
	if (flag == 1) {
		//visualization
		pcl::visualization::PCLVisualizer Viewer;
		pcl::visualization::PointCloudColorHandlerCustom<UserPointType> color_handler(cloud_in.makeShared(), 0, 255, 255);  //0, 229, 238
		Viewer.addPointCloud(cloud_in.makeShared(), color_handler, "cloud");
		Viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud");
		Viewer.addCoordinateSystem(100);
		Viewer.setBackgroundColor(255, 255, 255);
		while (!Viewer.wasStopped())
		{
			Viewer.spinOnce(100);
			//boost::this_thread::sleep(boost::posix_time::microseconds(100000));
		}
		Viewer.close();
		return 1;
	}
		//boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
		//viewer->setBackgroundColor(0, 0, 0);
		//viewer->addPointCloud<pcl::PointXYZ>(cloud_in.makeShared(), "sample cloud");
		////viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
		//viewer->addCoordinateSystem(0.1);
		//viewer->initCameraParameters();

		//while (!viewer->wasStopped())
		//{
		//	viewer->spinOnce(100);
		//	boost::this_thread::sleep(boost::posix_time::microseconds(100000));
		//}
		//viewer->close();
		//return 1;
}

/*��Ŀ��PCA*/
void ProcessCloud::RoughPCA(const pcl::PointCloud<UserPointType>& cloud_cluster, const UserPointType& sample_point, std::vector<float>& manipulator_posture) const {
	manipulator_posture.clear();

	auto start_time = std::chrono::steady_clock::now();

	Eigen::Vector4f pcaCentroid, pcaCentroid_wor;
	pcl::compute3DCentroid(cloud_cluster, pcaCentroid);
	Eigen::Matrix3f covariance;
	pcl::computeCovarianceMatrixNormalized(cloud_cluster, pcaCentroid, covariance);
	Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance, Eigen::ComputeEigenvectors);
	Eigen::Matrix3f eigenVectorsPCA = eigen_solver.eigenvectors();
	Eigen::Vector3f eigenValuesPCA = eigen_solver.eigenvalues();

	/*У��������䴹ֱ*/
	eigenVectorsPCA.col(2) = eigenVectorsPCA.col(0).cross(eigenVectorsPCA.col(1));  //��Ӧ�������ֵ
	eigenVectorsPCA.col(0) = eigenVectorsPCA.col(1).cross(eigenVectorsPCA.col(2));  //��Ӧ��С����ֵ
	eigenVectorsPCA.col(1) = eigenVectorsPCA.col(2).cross(eigenVectorsPCA.col(0));  //��Ӧ��ֵ����ֵ

	Eigen::Matrix4f tm = Eigen::Matrix4f::Identity();
	Eigen::Matrix4f tm_inv = Eigen::Matrix4f::Identity();
	tm.block<3, 3>(0, 0) = eigenVectorsPCA.transpose();   //R.
	tm.block<3, 1>(0, 3) = -1.0f * (eigenVectorsPCA.transpose()) * (pcaCentroid.head<3>());//  -R*t
	tm_inv = tm.inverse();

	pcl::PointCloud<UserPointType>::Ptr transformedCloud(new pcl::PointCloud<UserPointType>);
	pcl::transformPointCloud(cloud_cluster, *transformedCloud, tm);

	UserPointType min_p1, max_p1;
	Eigen::Vector3f c1, c, c_wor; //��������
	pcl::getMinMax3D(*transformedCloud, min_p1, max_p1);
	c1 = 0.5f * (min_p1.getVector3fMap() + max_p1.getVector3fMap());

	Eigen::Affine3f tm_inv_aff(tm_inv);
	Eigen::Affine3f tm_w2c_aff(param::wor2cam_trans);
	pcl::transformPoint(c1, c, tm_inv_aff);
	pcl::transformPoint(c, c_wor, tm_w2c_aff);

	Eigen::Vector3f whd, whd1, whd_temp;
	whd1 = max_p1.getVector3fMap() - min_p1.getVector3fMap();
	whd = whd1;
	float sc1 = (whd1(0) + whd1(1) + whd1(2)) / 3;  //����ƽ���߶ȣ����������������ͷ��С

	int min_index;
	whd1.minCoeff(&min_index);
	if (abs(eigenVectorsPCA(2, min_index)) > 0.5) {
		whd_temp = whd1;
		whd_temp(min_index) += whd1(0) + whd1(1) + whd1(2);
		whd_temp.minCoeff(&min_index);
	}
	float theta_5;
	if (eigenVectorsPCA(1, min_index) == 0)
		theta_5 = M_PI_2;
	else
		theta_5 = atan(eigenVectorsPCA(0, min_index) / eigenVectorsPCA(1, min_index));
	//if (theta_5 < 0)
	//	theta_5 += (M_PI * 2);

	/*�������ϵ��λ��ƫ��*/
	Eigen::Vector4f pcaCentroid_temp = pcaCentroid;
	pcaCentroid(0) += param::displace_x;  //x,y,z����
	pcaCentroid(1) += param::displace_y;
	pcaCentroid(2) += param::displace_z;
	//theta_5 += ((param::displace_theta) * M_PI / 180);  //������ƫ��

	theta_5 = (2048 / M_PI) * theta_5;  //������
	pcaCentroid_wor = (param::wor2cam_trans)*pcaCentroid;  //��������ϵ

	for (int i = 0; i < 3; i++)
		manipulator_posture.emplace_back(pcaCentroid(i));  //�������ϵ,����������ϵ
	manipulator_posture.emplace_back(theta_5);

	Eigen::Vector3f tempt = sample_point.getVector3fMap();
	float len = (c - sample_point.getVector3fMap()).norm();  //norm()�����ص��Ƿ��� ��squaredNorm()�ĸ�

	std::cout << "����ֵva(3x1):\n" << eigenValuesPCA << std::endl;
	std::cout << "��������ve(3x3):\n" << eigenVectorsPCA << std::endl;
	std::cout << "width1=" << whd1(0) << endl;
	std::cout << "heght1=" << whd1(1) << endl;
	std::cout << "depth1=" << whd1(2) << endl;
	std::cout << "scale1=" << sc1 << endl;
	std::cout << "��ȡ����Ϊ:\n" << eigenVectorsPCA.col(min_index) << std::endl;
	std::cout << "��Ƥ���н�Ϊ:\n" << theta_5 * 180 / 2048 << std::endl;
	std::cout << "�������������ϵ��λ��(4x1):\n" << pcaCentroid_temp << std::endl;
	std::cout << "��������������ϵ��λ��(4x1):\n" << pcaCentroid_wor << std::endl;
	std::cout << "�������ĵ�������ľ���:\n" << len << std::endl;
	std::cout << "�����뾶:" << param::kdtree_rad << std::endl;
	std::cout << "��������:" << c << std::endl;
	std::cout << "������:" << tempt << std::endl;

	if (0) {  //param::plot_pc
		const Eigen::Quaternionf bboxQ(eigenVectorsPCA);  //�����峤��߷���
		const Eigen::Vector3f    bboxT(c);  //�����弸������
		//��ʼ���Ƶ�������
		UserPointType cp;
		cp.x = pcaCentroid_temp(0);
		cp.y = pcaCentroid_temp(1);
		cp.z = pcaCentroid_temp(2);
		UserPointType pcX;
		pcX.x = sc1 * eigenVectorsPCA(0, 0) + cp.x;
		pcX.y = sc1 * eigenVectorsPCA(1, 0) + cp.y;
		pcX.z = sc1 * eigenVectorsPCA(2, 0) + cp.z;
		UserPointType pcY;
		pcY.x = sc1 * eigenVectorsPCA(0, 1) + cp.x;
		pcY.y = sc1 * eigenVectorsPCA(1, 1) + cp.y;
		pcY.z = sc1 * eigenVectorsPCA(2, 1) + cp.z;
		UserPointType pcZ;
		pcZ.x = sc1 * eigenVectorsPCA(0, 2) + cp.x;
		pcZ.y = sc1 * eigenVectorsPCA(1, 2) + cp.y;
		pcZ.z = sc1 * eigenVectorsPCA(2, 2) + cp.z;

		//��ȡ����
		UserPointType p(0, 0, 0);
		UserPointType vector(sc1 * eigenVectorsPCA(0, min_index), sc1 * eigenVectorsPCA(1, min_index), sc1 * eigenVectorsPCA(2, min_index));

		//visualization
		pcl::visualization::PCLVisualizer viewer;
		pcl::visualization::PointCloudColorHandlerCustom<UserPointType> color_handler(cloud_cluster.makeShared(), 0, 229, 238);  //����ĳ�ʼ�������
		viewer.addPointCloud(cloud_cluster.makeShared(), color_handler, "cloud");
		viewer.addCube(bboxT, bboxQ, whd(0), whd(1), whd(2), "bbox");
		viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, "bbox");
		viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 0, 1, "bbox");
		viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud");

		viewer.addArrow(pcX, cp, 1.0, 0.0, 0.0, false, "arrow_x");
		viewer.addArrow(pcY, cp, 0.0, 1.0, 0.0, false, "arrow_y");
		viewer.addArrow(pcZ, cp, 0.0, 0.0, 1.0, false, "arrow_z");
		//viewer.addArrow(vector, p, 0.0, 0.0, 0.0, false, "vector");

		//viewer.addCoordinateSystem(0.5f*sc1);
		viewer.setBackgroundColor(255, 255, 255);
		while (!viewer.wasStopped())
		{
			viewer.spinOnce(100);
		}
	}

	auto end_time = std::chrono::steady_clock::now();
	auto total_span = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
	std::cout << "Extract target pose runtime:" << total_span.count() << " ms" << std::endl;
}

/*����������������С����*/
bool ProcessCloud::RoughPCA(const pcl::PointCloud<UserPointType>& cloud_cluster, Eigen::Matrix3f& eigenVectorsPCA, float& value_ratio) const {
	auto start_time = std::chrono::steady_clock::now();

	Eigen::Vector4f pcaCentroid, pcaCentroid_wor;  //����
	pcl::compute3DCentroid(cloud_cluster, pcaCentroid);

	Eigen::Matrix3f covariance;  //Э�������
	pcl::computeCovarianceMatrixNormalized(cloud_cluster, pcaCentroid, covariance);
	Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance, Eigen::ComputeEigenvectors);

	Eigen::Vector3f eigenValuesPCA = eigen_solver.eigenvalues();  //����ֵ����С��������
	eigenVectorsPCA = eigen_solver.eigenvectors();  //��������
	value_ratio = (eigenValuesPCA(2) / eigenValuesPCA(1));

	/*У��������䴹ֱ*/
	eigenVectorsPCA.col(2) = eigenVectorsPCA.col(0).cross(eigenVectorsPCA.col(1));  //������ˣ���Ӧ�������ֵ
	eigenVectorsPCA.col(0) = eigenVectorsPCA.col(1).cross(eigenVectorsPCA.col(2));  //��Ӧ��С����ֵ
	eigenVectorsPCA.col(1) = eigenVectorsPCA.col(2).cross(eigenVectorsPCA.col(0));  //��Ӧ��ֵ����ֵ

	std::cout << "����ֵva(3x1):\n" << eigenValuesPCA << std::endl;
	std::cout << "��������ve(3x3):\n" << eigenVectorsPCA << std::endl;
	std::cout << "v3 / v2: " << value_ratio << std::endl;

	auto end_time = std::chrono::steady_clock::now();
	auto total_span = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
	std::cout << "PCA runtime:" << total_span.count() << " ms" << std::endl;

	if (0) {
		Eigen::Matrix4f tm = Eigen::Matrix4f::Identity();
		Eigen::Matrix4f tm_inv = Eigen::Matrix4f::Identity();
		tm.block<3, 3>(0, 0) = eigenVectorsPCA.transpose();   //R.
		tm.block<3, 1>(0, 3) = -1.0f * (eigenVectorsPCA.transpose()) * (pcaCentroid.head<3>());//  -R*t
		tm_inv = tm.inverse();

		pcl::PointCloud<UserPointType>::Ptr transformedCloud(new pcl::PointCloud<UserPointType>);
		pcl::transformPointCloud(cloud_cluster, *transformedCloud, tm);

		UserPointType min_p1, max_p1;
		Eigen::Vector3f c1, c, c_wor; //��������
		pcl::getMinMax3D(*transformedCloud, min_p1, max_p1);
		c1 = 0.5f * (min_p1.getVector3fMap() + max_p1.getVector3fMap());

		Eigen::Affine3f tm_inv_aff(tm_inv);
		Eigen::Affine3f tm_w2c_aff(param::wor2cam_trans);
		pcl::transformPoint(c1, c, tm_inv_aff);
		pcl::transformPoint(c, c_wor, tm_w2c_aff);

		Eigen::Vector3f whd, whd1, whd_temp;
		whd1 = max_p1.getVector3fMap() - min_p1.getVector3fMap();
		whd = whd1;
		float sc1 = (whd1(0) + whd1(1) + whd1(2)) / 3;  //����ƽ���߶ȣ����������������ͷ��С

		std::cout << "width1=" << whd1(0) << std::endl;
		std::cout << "heght1=" << whd1(1) << std::endl;
		std::cout << "depth1=" << whd1(2) << std::endl;
		std::cout << "search_radius=" << param::kdtree_rad << std::endl;

		const Eigen::Quaternionf bboxQ(eigenVectorsPCA);  //�����峤��߷���
		const Eigen::Vector3f  bboxT(c);  //�����弸������
		//��ʼ���Ƶ�������
		UserPointType cp;
		cp.x = pcaCentroid(0);
		cp.y = pcaCentroid(1);
		cp.z = pcaCentroid(2);
		UserPointType pcX;
		pcX.x = sc1 * eigenVectorsPCA(0, 0) + cp.x;
		pcX.y = sc1 * eigenVectorsPCA(1, 0) + cp.y;
		pcX.z = sc1 * eigenVectorsPCA(2, 0) + cp.z;
		UserPointType pcY;
		pcY.x = sc1 * eigenVectorsPCA(0, 1) + cp.x;
		pcY.y = sc1 * eigenVectorsPCA(1, 1) + cp.y;
		pcY.z = sc1 * eigenVectorsPCA(2, 1) + cp.z;
		UserPointType pcZ;
		pcZ.x = sc1 * eigenVectorsPCA(0, 2) + cp.x;
		pcZ.y = sc1 * eigenVectorsPCA(1, 2) + cp.y;
		pcZ.z = sc1 * eigenVectorsPCA(2, 2) + cp.z;

		//visualization
		pcl::visualization::PCLVisualizer viewer;
		pcl::visualization::PointCloudColorHandlerCustom<UserPointType> color_handler(cloud_cluster.makeShared(), 255, 255, 255);
		viewer.addPointCloud(cloud_cluster.makeShared(), color_handler, "cloud");//0, 229, 238
		viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud");
		viewer.addCube(bboxT, bboxQ, whd(0), whd(1), whd(2), "bbox");
		viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, 
			pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, "bbox");
		viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 0, 1, "bbox");

		viewer.addArrow(pcX, cp, 1.0, 0.0, 0.0, false, "arrow_x");
		viewer.addArrow(pcY, cp, 0.0, 1.0, 0.0, false, "arrow_y");
		viewer.addArrow(pcZ, cp, 0.0, 0.0, 1.0, false, "arrow_z");

		//viewer.addCoordinateSystem(0.5f*sc1);
		//viewer.setBackgroundColor(255, 255, 255);
		while (!viewer.wasStopped())
		{
			viewer.spinOnce(100);
		}
	}

	return !eigenVectorsPCA.isZero() && (value_ratio > 0);
}

bool ProcessCloud::ExtractBoundary(const pcl::PointCloud<UserPointType>& cloud_project, pcl::PointCloud<UserPointType>& cloud_boundary) const {

	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
	pcl::PointCloud<pcl::Boundary> boundaries;
	pcl::BoundaryEstimation<UserPointType, pcl::Normal, pcl::Boundary> est;
	pcl::search::KdTree<UserPointType>::Ptr tree(new pcl::search::KdTree<UserPointType>);

	CalNormalsOMP(cloud_project, tree, *normals);
	std::cout << "normal size is " << normals->size() << std::endl;

	/*�߽���ȡ*/
	est.setInputCloud(cloud_project.makeShared());
	est.setInputNormals(normals);
	est.setSearchMethod(tree);
	est.setKSearch(50);  //һ���������ֵԽ�ߣ����ձ߽�ʶ��ľ���Խ��
	est.compute(boundaries);

	int countBoundaries = 0;
	uint8_t bound_flag = 0;
	for (size_t i = 0; i < cloud_project.size(); i++) {
		bound_flag = (boundaries.points[i].boundary_point);
		//int a = static_cast<int>(x); //�ú����Ĺ�����ǿ������ת��
		if (bound_flag == 1)
		{
			cloud_boundary.emplace_back(cloud_project.points[i]);
			countBoundaries++;
		}
	}
	std::cout << "boudary size is��" << countBoundaries << std::endl;

	if (0) //param::plot_pc
		VisualizePC(cloud_boundary, 1);

	return countBoundaries > 0;
}

bool ProcessCloud::CalNormalsOMP(const pcl::PointCloud<UserPointType>& cloud_in, const pcl::search::KdTree<UserPointType>::Ptr& tree, 
	pcl::PointCloud<pcl::Normal>& normals) const {

	auto start_time = std::chrono::steady_clock::now();

	/*OMP��ⷨ��,�ٶ���NormalEstimation��3������*/
	pcl::NormalEstimationOMP<UserPointType, pcl::Normal> estOMP(4);
	estOMP.setInputCloud(cloud_in.makeShared());
	estOMP.setSearchMethod(tree);
	estOMP.setKSearch(9);
	estOMP.compute(normals);
	std::cout << "normal size is " << normals.size() << std::endl;

	auto end_time = std::chrono::steady_clock::now();
	auto total_span = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
	std::cout << "CalculateNormalsOMP runtime:" << total_span.count() << " ms" << std::endl;

	return normals.size();
}

bool ProcessCloud::CalCurvatureAxis(const pcl::PointCloud<UserPointType>& cloud_in, Eigen::Matrix3f& curvature_axis) const {
	auto start_time = std::chrono::steady_clock::now();
	pcl::PointCloud<pcl::Normal> normals;
	pcl::search::KdTree<UserPointType>::Ptr tree(new pcl::search::KdTree<UserPointType>);
	CalNormalsOMP(cloud_in, tree, normals);

	Eigen::Matrix3Xf normals_mat;
	normals_mat.resize(3, normals.size());

	for (size_t i = 0; i < normals.size(); i++) {
		normals_mat(0, i) = normals[i].normal_x;
		normals_mat(1, i) = normals[i].normal_y;
		normals_mat(2, i) = normals[i].normal_z;
	}

	Eigen::Matrix3f mat = normals_mat * normals_mat.transpose();
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(mat, Eigen::ComputeEigenvectors); 
    Eigen::Vector3f eigen_values = eigen_solver.eigenvalues();  //.real()
	curvature_axis = eigen_solver.eigenvectors();  //����ֵ��С����:������С��������󡢷���

	std::cout << "����ֵva(normals)(3x1):\n" << eigen_values << std::endl;
	std::cout << "��������ve(3x3)(normals):\n" << curvature_axis << std::endl;

	auto end_time = std::chrono::steady_clock::now();
	auto total_span = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
	std::cout << "CalCurvatureAxis runtime:" << total_span.count() << " ms" << std::endl;

	//curvature_axis.col(0) = eigen_vectors.col(0).normalized();  //��һ��
	//curvature_axis.col(1) = eigen_vectors.col(1).normalized();
	//curvature_axis.col(2) = eigen_vectors.col(2).normalized();

	if (param::plot_pc) {
		Eigen::Vector4f pcaCentroid;  //����
		pcl::compute3DCentroid(cloud_in, pcaCentroid);

		float sc1 = 10;  //����ƽ���߶ȣ����������������ͷ��С

		//��ʼ���Ƶ�������
		UserPointType cp;
		cp.x = pcaCentroid(0);
		cp.y = pcaCentroid(1);
		cp.z = pcaCentroid(2);
		UserPointType pcX;
		pcX.x = sc1 * curvature_axis(0, 0) + cp.x;
		pcX.y = sc1 * curvature_axis(1, 0) + cp.y;
		pcX.z = sc1 * curvature_axis(2, 0) + cp.z;
		UserPointType pcY;
		pcY.x = sc1 * curvature_axis(0, 1) + cp.x;
		pcY.y = sc1 * curvature_axis(1, 1) + cp.y;
		pcY.z = sc1 * curvature_axis(2, 1) + cp.z;
		UserPointType pcZ;
		pcZ.x = sc1 * curvature_axis(0, 2) + cp.x;
		pcZ.y = sc1 * curvature_axis(1, 2) + cp.y;
		pcZ.z = sc1 * curvature_axis(2, 2) + cp.z;

		//visualization
		pcl::visualization::PCLVisualizer viewer;
		pcl::visualization::PointCloudColorHandlerCustom<UserPointType> color_handler(cloud_in.makeShared(), 255, 255, 255);  
		viewer.addPointCloud(cloud_in.makeShared(), color_handler, "cloud");
		viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud");

		viewer.addArrow(pcX, cp, 1.0, 0.0, 0.0, false, "arrow_x");
		viewer.addArrow(pcY, cp, 0.0, 1.0, 0.0, false, "arrow_y");
		viewer.addArrow(pcZ, cp, 0.0, 0.0, 1.0, false, "arrow_z");

		//viewer.addCoordinateSystem(0.5f*sc1);
		viewer.setBackgroundColor(0, 0, 0);
		while (!viewer.wasStopped())
		{
			viewer.spinOnce(100);
		}
	}
	return (!curvature_axis.isZero());
}

/*�ٽ�������*/
bool ProcessCloud::RadiusSearch(const float& search_radius, const int& min_pc, const UserPointType& searchPoint,
	const pcl::PointCloud<UserPointType>& cloud_filtered, pcl::PointCloud<UserPointType>& cloud_cluster) const {
	cloud_cluster.clear();

	pcl::KdTreeFLANN<UserPointType> kdtree;
	kdtree.setInputCloud(cloud_filtered.makeShared());

	std::vector<int> pointIdxRadiusSearch;
	std::vector<float> pointRadiusSquaredDistance;

	if (kdtree.radiusSearch(searchPoint, search_radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > min_pc)
	{
		for (size_t i = 0; i < pointIdxRadiusSearch.size(); ++i)
			cloud_cluster.points.emplace_back(cloud_filtered.points[pointIdxRadiusSearch[i]]); //cloud_clusterΪ�������ӵ���

		if (0)
			VisualizePC(cloud_cluster, 1);

		return cloud_cluster.size() > 0;
	}
	else {
		std::cout << "The number of pc is less than " << min_pc << std::endl;
		return false;
	}
}

void ProcessCloud::LineFit(const pcl::PointCloud<UserPointType>& cloud_in, pcl::ModelCoefficients::Ptr& coefficients) const {
	//����һ��ģ�Ͳ��������������ֱ��
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices);  //inliers��ʾ��������̵ĵ� ��¼���ǵ��Ƶ����
	pcl::SACSegmentation<UserPointType> seg;     // ����һ���ָ���

	seg.setOptimizeCoefficients(true);     // Optional��������ÿ���ѡ�����ƽ��չʾ�ĵ��Ƿָ���ĵ㻹�Ƿָ�ʣ�µĵ㡣
	seg.setModelType(pcl::SACMODEL_LINE);  // Mandatory-����Ŀ�꼸����״
	seg.setMethodType(pcl::SAC_RANSAC);    //�ָ�������������
	seg.setDistanceThreshold(param::distance_threshold);         //����������̷�Χ��Ҳ������ֵ
	seg.setInputCloud(cloud_in.makeShared()); //�������
	seg.segment(*inliers, *coefficients);
}

/*���ط����Ʋ���*/
bool ProcessCloud::SampleVoxel(const pcl::PointCloud<UserPointType>& cloud_filtered, pcl::PointCloud<UserPointType>& cloud_samples) const {
	pcl::VoxelGrid<UserPointType> filter;
	filter.setInputCloud(cloud_filtered.makeShared());
	filter.setLeafSize(param::sample_leaf, param::sample_leaf, param::sample_leaf);
	filter.filter(cloud_samples);
	std::cout << "cloud_samples: " << cloud_samples.size() << std::endl;

	if (param::plot_pc)
		VisualizePC(cloud_samples, 1);

	return cloud_samples.size() > 0;
}

void ProcessCloud::ReadDataSet(pcl::PointCloud<UserPointType>& cloud_filtered) {
	cloud_filtered.clear();
	pcl::PointCloud<UserPointType>::Ptr cloud(new pcl::PointCloud<UserPointType>);

	pcl::io::loadPLYFile<UserPointType>(param::ply_path, *cloud);
	//pcl::io::loadPCDFile<UserPointType>(param::pcd_path, *cloud);
	Eigen::Matrix4f mat;
	mat << 1, 0, 0, 0,
		0, -1, 0, 0,
		0, 0, -1, 1.5,
		0, 0, 0, 1;

	pcl::PointCloud<UserPointType>::Ptr cloud_transform(new pcl::PointCloud<UserPointType>);
	pcl::transformPointCloud(*cloud, *cloud_transform, mat);

	Eigen::Vector4f pcaCentroid;
	pcl::compute3DCentroid(*cloud, pcaCentroid);
	Eigen::Matrix3f covariance;
	pcl::computeCovarianceMatrixNormalized(*cloud, pcaCentroid, covariance);
	Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance, Eigen::ComputeEigenvectors);
	Eigen::Matrix3f eigenVectorsPCA = eigen_solver.eigenvectors();
	Eigen::Vector3f eigenValuesPCA = eigen_solver.eigenvalues();

	/*У��������䴹ֱ*/
	eigenVectorsPCA.col(2) = eigenVectorsPCA.col(0).cross(eigenVectorsPCA.col(1));  //��Ӧ�������ֵ
	eigenVectorsPCA.col(0) = eigenVectorsPCA.col(1).cross(eigenVectorsPCA.col(2));  //��Ӧ��С����ֵ
	eigenVectorsPCA.col(1) = eigenVectorsPCA.col(2).cross(eigenVectorsPCA.col(0));  //��Ӧ��ֵ����ֵ

	Eigen::Matrix4f tm = Eigen::Matrix4f::Identity();
	Eigen::Matrix4f tm_inv = Eigen::Matrix4f::Identity();
	tm.block<3, 3>(0, 0) = eigenVectorsPCA.transpose();   //R.
	tm.block<3, 1>(0, 3) = -1.0f * (eigenVectorsPCA.transpose()) * (pcaCentroid.head<3>());//  -R*t
	tm_inv = tm.inverse();

	pcl::PointCloud<UserPointType>::Ptr transformedCloud(new pcl::PointCloud<UserPointType>);
	pcl::transformPointCloud(*cloud, *transformedCloud, tm);

	UserPointType min_p1, max_p1;
	Eigen::Vector3f c1, c; //��������
	pcl::getMinMax3D(*transformedCloud, min_p1, max_p1);
	c1 = 0.5f * (min_p1.getVector3fMap() + max_p1.getVector3fMap());

	Eigen::Affine3f tm_inv_aff(tm_inv);
	pcl::transformPoint(c1, c, tm_inv_aff);

	Eigen::Vector3f whd1;
	whd1 = max_p1.getVector3fMap() - min_p1.getVector3fMap();
	float sc1 = (whd1(0) + whd1(1) + whd1(2)) / 3;  //����ƽ���߶ȣ����������������ͷ��С
	float scal = (75 / sc1);
	std::cout << "scal:" << scal << std::endl;
	for (auto point : cloud->points) {     
		cloud_filtered.points.emplace_back(UserPointType(point.x * scal,
			point.y * scal, point.z * scal)); 
	}

	if (param::plot_pc)
		VisualizePC(cloud_filtered, 1);
	//VisualizePC(cloud_filtered, 1);
	///*�����˲�������*/
	//pcl::PassThrough<UserPointType> pass;
	//pass.setInputCloud(cloud_filtered.makeShared());
	//pass.setFilterFieldName("z");
	//pass.setFilterLimits(param::workspace[4], param::workspace[5]);  //��λ��mm,�������Ч����Ϊ340-440mm
	//pass.filter(cloud_filtered);

	//VisualizePC(cloud_filtered, 1);


	//cloud_filtered.width = cloud_filtered.size();
	//cloud_filtered.height = 1;
	//cloud_filtered.resize(cloud_filtered.size());
	//pcl::io::savePLYFileASCII("D:\\Doraemon\\pcd_qxc\\YCB\\bowl_filitered.ply", cloud_filtered);
	//std::cout << "Train dataset has been saved!" << std::endl;
}

bool ProcessCloud::CalAngleY(const pcl::PointCloud<UserPointType>& cloud_fixture, float& theta_left, float& theta_right) {
	pcl::PointCloud<UserPointType> cloud_project(cloud_fixture);
	pcl::PointCloud<UserPointType>::Ptr cloud_boundary(new pcl::PointCloud<UserPointType>);
	pcl::PointCloud<UserPointType>::Ptr cloud_boundary_left(new pcl::PointCloud<UserPointType>);
	pcl::PointCloud<UserPointType>::Ptr cloud_boundary_right(new pcl::PointCloud<UserPointType>);

	for (size_t i = 0; i < cloud_fixture.size(); i++) {
		cloud_project.points[i].x = 0;
	}

	ExtractBoundary(cloud_project, *cloud_boundary);

	pcl::PassThrough<UserPointType> pass;
	/*��ȡֱ��*/
	pass.setInputCloud(cloud_boundary);
	pass.setFilterFieldName("z");
	pass.setFilterLimits(-param::finger_z + param::filterlimits_z, param::finger_z - param::filterlimits_z);  //��λ��mm
	pass.filter(*cloud_boundary);

	if (0) //param::plot_pc
		VisualizePC(*cloud_boundary, 1);

	pass.setInputCloud(cloud_boundary);
	pass.setFilterFieldName("y");
	pass.setFilterLimits(-param::finger_y, 0);
	pass.filter(*cloud_boundary_left);

	if (0) //param::plot_pc
		VisualizePC(*cloud_boundary_left, 1);

	pass.setInputCloud(cloud_boundary);
	pass.setFilterFieldName("y");
	pass.setFilterLimits(0, param::finger_y);
	pass.filter(*cloud_boundary_right);

	if (0) //param::plot_pc
		VisualizePC(*cloud_boundary_right, 1);

	if (cloud_boundary_left->size() < 3 || cloud_boundary_right->size() < 3) {  //����ȷ��һ��ֱ��
		std::cout << "Error::Too few boundary points!" << std::endl;
		return 0;
	}

	//����һ��ģ�Ͳ��������������ֱ��
	pcl::ModelCoefficients::Ptr coefficients_left(new pcl::ModelCoefficients);
	pcl::ModelCoefficients::Ptr coefficients_right(new pcl::ModelCoefficients);
	
	LineFit(*cloud_boundary_right, coefficients_right);
	LineFit(*cloud_boundary_left, coefficients_left);

	float x1(coefficients_left->values[0]), y1(coefficients_left->values[1]), z1(coefficients_left->values[2]),
		Vx1(coefficients_left->values[3]), Vy1(coefficients_left->values[4]), Vz1(coefficients_left->values[5]),
		x2(coefficients_right->values[0]), y2(coefficients_right->values[1]), z2(coefficients_right->values[2]),
		Vx2(coefficients_right->values[3]), Vy2(coefficients_right->values[4]), Vz2(coefficients_right->values[5]);

	if (Vz1 < 0) {
		Vx1 = -Vx1;
		Vy1 = -Vy1;
		Vz1 = -Vz1;
	}
	if (Vz2 < 0) {
		Vx2 = -Vx2;
		Vy2 = -Vy2;
		Vz2 = -Vz2;
	}
	theta_left = atan2(Vz1, Vy1);  //��ϵ�ֱ����y��ļн�,������,[0,pi]
	theta_right = atan2(Vz2, Vy2);

	if (0) {
		UserPointType pcA(x1 - 5 * Vx1, y1 - 5 * Vy1, z1 - 5 * Vz1);
		UserPointType pcB(x1 + 5 * Vx1, y1 + 5 * Vy1, z1 + 5 * Vz1);
		UserPointType pcC(x2 - 5 * Vx2, y2 - 5 * Vy2, z2 - 5 * Vz2);
		UserPointType pcD(x2 + 5 * Vx2, y2 + 5 * Vy2, z2 + 5 * Vz2);

		// ���ƿ��ӻ�
		pcl::visualization::PCLVisualizer viewer;
		pcl::visualization::PointCloudColorHandlerCustom<UserPointType> color_handler(cloud_project.makeShared(), 255, 255, 255);  //����ĳ�ʼ�������
		viewer.addPointCloud(cloud_project.makeShared(), color_handler, "cloud");
		viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud");
		viewer.addLine(pcA, pcB, 255, 0, 0, "line_left");
		viewer.addLine(pcC, pcD, 255, 0, 0, "line_right");
		//viewer.setBackgroundColor(255, 255, 255);
		while (!viewer.wasStopped())
			viewer.spinOnce(100);
	}
	return true;
}
