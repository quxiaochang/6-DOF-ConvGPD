#pragma warning(disable:4996)
#include "grasp.h"

GraspSet::GraspSet(const UserPointType& sample_point_, const UserPointType& centroid_point_) {
	sample_point = sample_point_;
	centroid_point = centroid_point_;
	displace_x = param::displace_x;
}

/*计算夹具X轴向量与抓取点*/
bool GraspSet::CalcuAxisXGrasp(Eigen::Vector3f& normal_vector) {
	/*矫正法线, 法线指向内部*/
	float vector_dot = 0;
	if (param::dataset_test)
		vector_dot = normal_vector.dot(centroid_point.getVector3fMap() - sample_point.getVector3fMap());  //向量点乘，代表|a||b|cosθ
	else {
		Eigen::Vector3f view_vector(sample_point.x, sample_point.y, (sample_point.z - 500)); //LMI相机校准之后视角点的大致位置为(0,0,500)
		vector_dot = normal_vector.dot(view_vector);
	}
	std::cout << "点乘获取|a||b|cosθ:" << vector_dot << std::endl;

	if (vector_dot < 0) {
		normal_vector *= -1;
		std::cout << "矫正之后的特征向量ve(3x3):\n" << normal_vector << std::endl;
	}

	fix_axis_x = normal_vector.normalized();  //法线向量归一化
	float x_norm = fix_axis_x.norm();
	std::cout << "fix_axis_x:" << fix_axis_x << std::endl;
	std::cout << "fix_axis_x.norm:" << x_norm << std::endl;

	/*初始化抓取点，抓取点相对于与采样点(坐标系原点)偏移displace_x*/
	grasp_point.x = displace_x * fix_axis_x(0) + sample_point.x;
	grasp_point.y = displace_x * fix_axis_x(1) + sample_point.y;
	grasp_point.z = displace_x * fix_axis_x(2) + sample_point.z;

	return (!fix_axis_x.isZero());
}

/*计算夹具YZ轴向量和变换矩阵*/
bool GraspSet::CalcuMatAxisYZ(Eigen::Vector3f& princom_vector) {
	princom_vector.normalize();  
	if (fix_axis_x == princom_vector || fix_axis_x == (-princom_vector)) {
		std::cout << "fix_axis_x and princom_vector are in the same direction!" << std::endl;
		return false;
	}
	else {
		fix_axis_y = fix_axis_x.cross(princom_vector); //ab向量叉乘，即求ab的垂直向量
		fix_axis_y.normalize();  //归一化

		float y_norm = fix_axis_y.norm();
		std::cout << "fix_axis_y:" << fix_axis_y << std::endl;
		std::cout << "fix_axis_y.norm:" << y_norm << std::endl;

		fix_axis_z = fix_axis_x.cross(fix_axis_y); //向量叉乘，顺序不能互换
		fix_axis_z.normalize();
		float z_norm = fix_axis_z.norm();
		std::cout << "fix_axis_z:" << fix_axis_z << std::endl;
		std::cout << "fix_axis_z.norm:" << z_norm << std::endl;

		CalcuTranMatrix();  

		return(!fix_axis_y.isZero() && !fix_axis_z.isZero());
	}
}

/*利用SVD方法求解夹具坐标系到相机坐标系的变换矩阵*/
void GraspSet::CalcuTranMatrix() {

	auto start_time = std::chrono::steady_clock::now();
	pcl::PointCloud<UserPointType>::Ptr cloud_in(new pcl::PointCloud<UserPointType>());
	pcl::PointCloud<UserPointType>::Ptr cloud_out(new pcl::PointCloud<UserPointType>());

	cloud_in->width = 4;
	cloud_in->height = 1;
	cloud_in->is_dense = false;
	cloud_in->resize(4);

	cloud_out->width = 4;
	cloud_out->height = 1;
	cloud_out->is_dense = false;
	cloud_out->resize(4);

	/*输入相机坐标系的点*/
	cloud_in->points[0] = (UserPointType(sample_point.x, sample_point.y, sample_point.z));                                                 //原点
	cloud_in->points[1] = (UserPointType(sample_point.x + fix_axis_x(0), sample_point.y + fix_axis_x(1), sample_point.z + fix_axis_x(2))); //X轴
	cloud_in->points[2] = (UserPointType(sample_point.x + fix_axis_y(0), sample_point.y + fix_axis_y(1), sample_point.z + fix_axis_y(2))); //Y轴
	cloud_in->points[3] = (UserPointType(sample_point.x + fix_axis_z(0), sample_point.y + fix_axis_z(1), sample_point.z + fix_axis_z(2))); //Z轴

	/*输出夹爪坐标系的对应点*/
	cloud_out->points[0] = (UserPointType(0, 0, 0));
	cloud_out->points[1] = (UserPointType(1, 0, 0));
	cloud_out->points[2] = (UserPointType(0, 1, 0));
	cloud_out->points[3] = (UserPointType(0, 0, 1));

	//利用SVD方法求解变换矩阵  
	pcl::registration::TransformationEstimationSVD<UserPointType, UserPointType> TESVD;
	//pcl::registration::TransformationEstimationSVD<UserPointType, UserPointType>::Matrix4 transformation2;
	
	TESVD.estimateRigidTransformation(*cloud_in, *cloud_out, fix2cam_mat);

	auto end_time = std::chrono::steady_clock::now();
	auto total_span = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
	std::cout << "CalcuTranMatrix runtime:" << total_span.count() << " ms" << std::endl;

	//输出变换矩阵信息  
	std::cout << "The Estimated Rotation and translation matrices (using getTransformation function) are : \n" << fix2cam_mat << std::endl ;
}

bool GraspSet::Algorithm1(Eigen::Vector3f& normal_vector, Eigen::Vector3f& princom_vector) {
	if (!CalcuAxisXGrasp(normal_vector))  //获取夹具X轴向量与抓取点
		return false;

	OptimizeTest(param::optimize_test, princom_vector); //优化检测

	if (!CalcuMatAxisYZ(princom_vector))  //获取夹具YZ轴向量与变换矩阵
		return false;

	return true;
}


bool GraspSet::Algorithm2(const Eigen::Matrix3f& curvature_axis) {
	Eigen::Vector3f curmin_vector = curvature_axis.col(0);
	Eigen::Vector3f curmax_vector = curvature_axis.col(1);
	Eigen::Vector3f normal_vector = curvature_axis.col(2);

	displace_x = param::displace_x;  //Algorithm1优化时修改了默认的值
	if (!CalcuAxisXGrasp(curmax_vector))  //curmax_vector作为夹具X轴，或者curmin_vector
		return false;

	if (!CalcuMatAxisYZ(normal_vector))  //curmin_vector，获取夹具YZ轴向量与变换矩阵,Y轴确定为normal_vector
		return false;

	return true;
}

void GraspSet::FiltFixturePC(const pcl::PointCloud<UserPointType>& cloud_filtered, pcl::PointCloud<UserPointType>& cloud_fixture, size_t& pc_arm) {
	cloud_fixture.clear();

	pcl::PointCloud<UserPointType>::Ptr cloud_transform(new pcl::PointCloud<UserPointType>); //转换到夹具坐标系														
	pcl::transformPointCloud(cloud_filtered, *cloud_transform, fix2cam_mat); //cloud_transform = mat* cloud_filtered

	pcl::PointCloud<UserPointType>::Ptr cloud_arm(new pcl::PointCloud<UserPointType>);
	pcl::PassThrough<UserPointType> pass;
	/*转换之后直通滤波*/
	pass.setInputCloud(cloud_transform);
	pass.setFilterFieldName("z");
	pass.setFilterLimits(-param::finger_z, param::finger_z);  //单位：mm
	pass.filter(cloud_fixture);

	pass.setInputCloud(cloud_fixture.makeShared());
	pass.setFilterFieldName("y");
	pass.setFilterLimits(-param::finger_y, param::finger_y);
	pass.filter(cloud_fixture);

	pass.setInputCloud(cloud_fixture.makeShared());
	pass.setFilterFieldName("x");
	pass.setFilterLimits(displace_x - 100, displace_x);
	pass.filter(*cloud_arm);

	pass.setInputCloud(cloud_fixture.makeShared());
	pass.setFilterFieldName("x");
	pass.setFilterLimits(displace_x, (2 * param::finger_x + displace_x));
	pass.filter(cloud_fixture);

	pc_arm = cloud_arm->size();
	std::cout << "Points in the fixture : " << cloud_fixture.size() << std::endl;
	std::cout << "Points in the arm : " << pc_arm << std::endl;

	if (param::plot_pc) {
		//Plot3DGrasp(cloud_filtered, cloud_fixture);
		Plot6DGrasp(*cloud_transform, cloud_fixture);
	}
}

bool GraspSet::OptimizeGrasp(const pcl::PointCloud<UserPointType>& cloud_filtered, pcl::PointCloud<UserPointType>& cloud_fixture, bool rotate_flag, ProcessCloud& pc) {
	cloud_fixture.clear();
	std::cout << "Start to optimize grasp!" << std::endl;
	size_t pc_arm(0);
	FiltFixturePC(cloud_filtered, cloud_fixture, pc_arm);//cloud_filtered必须是预处理ProcessPC之后的点云

	if (pc.SegmentPC(cloud_fixture) > 1 || grasp_point.z < (2 * param::finger_x)) { //不发生碰撞返回true!ClosureCollisionDetect(cloud_fixture)
		OptimizeGraspDisplace(pc);//位移优化graspset，修改displace_x和grasp_point，其他不变
		FiltFixturePC(cloud_filtered, cloud_fixture, pc_arm);
	}
	if (rotate_flag && PreScreenCand(cloud_fixture, pc_arm)) {
		OptimizeGraspRotate(cloud_fixture, pc);  //旋转优化graspset，变化与位移优化相反
		FiltFixturePC(cloud_filtered, cloud_fixture, pc_arm);
	}

	return PreScreenCand(cloud_fixture, pc_arm);
}

void GraspSet::OptimizeGraspDisplace(ProcessCloud& pc) {
	//if (pc.SegmentPC(cloud_fixture) > 1) {
	//	std::cout << "There is more than one pc in cloud_fixture" << std::endl;
	std::cout << "Start to optimize grasp displace!" << std::endl;
		float scl = 10;//沿x轴反方向位移，即sample_point离末端10mm
		displace_x = scl - 2 * param::finger_x; //修改displace_x
		grasp_point.x = displace_x * fix_axis_x(0) + sample_point.x;
		grasp_point.y = displace_x * fix_axis_x(1) + sample_point.y;
		grasp_point.z = displace_x * fix_axis_x(2) + sample_point.z;

		//return (!fix_axis_x.isZero()) && (!fix_axis_y.isZero()) && (!fix_axis_z.isZero());
	//}
	//else {
	//	std::cout << "There is only one pc in cloud_fixture" << std::endl;
	//	return true;
	//}

}
void GraspSet::OptimizeGraspRotate(const pcl::PointCloud<UserPointType>& cloud_fixture, ProcessCloud& pc) {
	std::cout << "Start to optimize grasp rotate" << std::endl;

	Eigen::Matrix4f ofix2fix_rotate = Eigen::Matrix4f::Identity();   //旋转矩阵
	
	float theta_left(0), theta_right(0), theta_x(0); //拟合的直线与y轴的夹角,弧度制,[0,pi]
	if (pc.CalAngleY(cloud_fixture, theta_left, theta_right)) {
		theta_x = (theta_left - M_PI + theta_right) / 2; //右手法则，原夹具坐标系绕x轴旋转的角度

		if (param::force_closure_test) {
			std::cout << "a1 before optimization: " << abs(theta_left - M_PI_2) / M_PI * 180 << std::endl;
			std::cout << "a2 before optimization:: " << abs(theta_right - M_PI_2) / M_PI * 180 << std::endl;
		}
		ofix2fix_rotate << 1, 0, 0, 0,
			0, cos(theta_x), sin(theta_x), 0,
			0, -sin(theta_x), cos(theta_x), 0,
			0, 0, 0, 1;
		fix2cam_mat = ofix2fix_rotate * fix2cam_mat;  //优化之后的夹具坐标系转换到之前夹具坐标系，再转换到相机坐标系

		std::cout << "The Estimated Rotation and translation matrices which are optimized : \n" << fix2cam_mat << std::endl;

		fix_axis_x << fix2cam_mat(0, 0), fix2cam_mat(0, 1), fix2cam_mat(0, 2);
		fix_axis_y << fix2cam_mat(1, 0), fix2cam_mat(1, 1), fix2cam_mat(1, 2);
		fix_axis_z << fix2cam_mat(2, 0), fix2cam_mat(2, 1), fix2cam_mat(2, 2);

		std::cout << "optimize fix_axis_x: \n" << fix_axis_x << std::endl;
		std::cout << "optimize fix_axis_y: \n" << fix_axis_y << std::endl;
		std::cout << "optimize fix_axis_z: \n" << fix_axis_z << std::endl;
	}
	//return (!fix_axis_x.isZero()) && (!fix_axis_y.isZero()) && (!fix_axis_z.isZero());
}

bool GraspSet::OptimizeTest(const bool& optimize_test, Eigen::Vector3f& principal_vector) {
	
	if (optimize_test) {
		Eigen::Matrix4f ofix2fix_mat;

		float theta_x(M_PI / 6);
		ofix2fix_mat << 1, 0, 0, 0,
			0, cos(theta_x), sin(theta_x), 0,
			0, -sin(theta_x), cos(theta_x), 0,
			0, 0, 0, 1;
		Eigen::Affine3f tm_inv_aff(ofix2fix_mat);

		pcl::transformPoint(principal_vector, principal_vector, tm_inv_aff);
		std::cout << "Start to test optimization" << std::endl;
	}
	else {
		std::cout << "Not test optimization" << std::endl;
	}

	return !principal_vector.isZero();
}

/*点云与夹具封闭区域碰撞检测，不发生碰撞返回true*/
bool GraspSet::ClosureCollisionDetect(const pcl::PointCloud<UserPointType>& cloud_fixture) {
	/*获取夹取宽度*/
	UserPointType min_p1, max_p1;  //碰撞检测依赖
	pcl::getMinMax3D(cloud_fixture, min_p1, max_p1);

	std::cout <<"min_p1: "<< min_p1 << max_p1 << std::endl;
	float coll_coeff = 0.9;  //不能为1，否则判断不准

	return (min_p1.x > (coll_coeff * displace_x)) &&
		(min_p1.y > (coll_coeff * (-param::finger_y))) &&
		(max_p1.y < (coll_coeff* param::finger_y));
}

/*点云与机械臂碰撞检测，不发生碰撞返回true*/
bool GraspSet::ArmCollisionDetect(const size_t& cloud_arm) {
	return  cloud_arm < 10;
}

/*判断夹具内部点云是否足够多，达到阈值返回true*/
bool GraspSet::PCNumberDetect(const pcl::PointCloud<UserPointType>& cloud_fixture) {
	int min_pc_fix(0);
	if (param::dataset_test)  //YCB等数据集点云相对稀疏，故设定阈值小一点
		min_pc_fix = 10;
	else
		min_pc_fix = 50;

	return cloud_fixture.size() > min_pc_fix;
}

/*判断抓取位姿的高度与重力向量的夹角*/
bool GraspSet::PostureDetect() {
	Eigen::Vector3f gravity_vector(0, 0, -1); //重力垂线向量
	double cos_val = gravity_vector.dot(fix_axis_x) / (gravity_vector.norm() * fix_axis_x.norm()); //角度cos值
	double angle = acos(cos_val) * 180 / M_PI;     //返回[0,pi]之间的计算结果，单位为弧度
	std::cout << "The angle between the X axis and the gravity vector: " <<angle << std::endl;
	return angle < 90 && grasp_point.z > (2 * param::finger_x);  //抓取位姿与重力向量的夹角、高度判断
}

/*抓取位姿初步筛选*/
bool GraspSet::PreScreenCand(const pcl::PointCloud<UserPointType>& cloud_fixture, const size_t& cloud_arm) {
	return PostureDetect() && PCNumberDetect(cloud_fixture) &&
		ArmCollisionDetect(cloud_arm) && ClosureCollisionDetect(cloud_fixture);
}

void GraspSet::PlotNorm(const pcl::PointCloud<UserPointType>& cloud_filtered) {
	UserPointType pcX;
	float sc = 20;
	pcX.x = -sc * fix_axis_x(0) + sample_point.x;
	pcX.y = -sc * fix_axis_x(1) + sample_point.y;
	pcX.z = -sc * fix_axis_x(2) + sample_point.z;

	pcl::visualization::PCLVisualizer Viewer;
	pcl::visualization::PointCloudColorHandlerCustom<UserPointType> color_handler(cloud_filtered.makeShared(), 0, 229, 238);
	Viewer.addPointCloud(cloud_filtered.makeShared(), color_handler, "cloud_filtered");
	Viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud_filtered");
	Viewer.addSphere(sample_point, 0.8, 255, 0, 0, "sample_point");
	Viewer.addLine(sample_point, pcX, 0, 255, 0, "normal");
	Viewer.setBackgroundColor(255, 255, 255);
	while (!Viewer.wasStopped())
	{
		Viewer.spinOnce(100);
	}
}

void GraspSet::Plot3DGrasp(const pcl::PointCloud<UserPointType>& cloud_filtered, const pcl::PointCloud<UserPointType>& cloud_fixture) {
	float sc1 = 20;  //sc1：坐标系长度，displace_x：抓取点上移距离
	UserPointType cp = sample_point;
	UserPointType pcX, pcY, pcZ;
	pcX.x = sc1 * fix_axis_x(0) + cp.x;
	pcX.y = sc1 * fix_axis_x(1) + cp.y;
	pcX.z = sc1 * fix_axis_x(2) + cp.z;

	pcY.x = sc1 * fix_axis_y(0) + cp.x;
	pcY.y = sc1 * fix_axis_y(1) + cp.y;
	pcY.z = sc1 * fix_axis_y(2) + cp.z;

	pcZ.x = sc1 * fix_axis_z(0) + cp.x;
	pcZ.y = sc1 * fix_axis_z(1) + cp.y;
	pcZ.z = sc1 * fix_axis_z(2) + cp.z;

	UserPointType pcA, pcB, pcC, pcD, pcE, pcF;
	pcA = grasp_point;

	pcB.x = (displace_x - 20) * fix_axis_x(0) + cp.x;
	pcB.y = (displace_x - 20) * fix_axis_x(1) + cp.y;
	pcB.z = (displace_x - 20) * fix_axis_x(2) + cp.z;

	pcC.x = -param::finger_y * fix_axis_y(0) + pcA.x;
	pcC.y = -param::finger_y * fix_axis_y(1) + pcA.y;
	pcC.z = -param::finger_y * fix_axis_y(2) + pcA.z;

	pcD.x = 2 * param::finger_x * fix_axis_x(0) + pcC.x;
	pcD.y = 2 * param::finger_x * fix_axis_x(1) + pcC.y;
	pcD.z = 2 * param::finger_x * fix_axis_x(2) + pcC.z;

	pcE.x = param::finger_y * fix_axis_y(0) + pcA.x;
	pcE.y = param::finger_y * fix_axis_y(1) + pcA.y;
	pcE.z = param::finger_y * fix_axis_y(2) + pcA.z;

	pcF.x = 2 * param::finger_x * fix_axis_x(0) + pcE.x;
	pcF.y = 2 * param::finger_x * fix_axis_x(1) + pcE.y;
	pcF.z = 2 * param::finger_x * fix_axis_x(2) + pcE.z;

	pcl::PointCloud<UserPointType>::Ptr cloud_camera(new pcl::PointCloud<UserPointType>);
	Eigen::Matrix4f tm_inv = Eigen::Matrix4f::Identity();  //初始化为单位矩阵
	tm_inv = fix2cam_mat.inverse();
	pcl::transformPointCloud(cloud_fixture, *cloud_camera, fix2cam_mat.inverse());  //将夹具坐标系点云转换回去

	pcl::visualization::PCLVisualizer Viewer;
	pcl::visualization::PointCloudColorHandlerCustom<UserPointType> color_handler1(cloud_filtered.makeShared(), 0, 255, 255);//0, 229, 238
	pcl::visualization::PointCloudColorHandlerCustom<UserPointType> color_handler2(cloud_camera, 255, 0, 255);
	Viewer.addPointCloud(cloud_filtered.makeShared(), color_handler1, "cloud_filtered");
	Viewer.addPointCloud(cloud_camera, color_handler2, "cloud_camera");
	Viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud_filtered");
	Viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud_camera");
	Viewer.addSphere(cp, 0.8, 255, 0, 0, "sample_point");

	/*添加坐标系*/
	Viewer.addArrow(pcX, cp, 1.0, 0.0, 0.0, false, "arrow_x");
	Viewer.addArrow(pcY, cp, 0.0, 1.0, 0.0, false, "arrow_y");
	Viewer.addArrow(pcZ, cp, 0.0, 0.0, 1.0, false, "arrow_z");

	/*添加抓取位姿*/
	Viewer.addLine(pcA, pcB, 255, 0, 0, "AB");
	Viewer.addLine(pcC, pcE, 255, 0, 0, "CE");
	Viewer.addLine(pcC, pcD, 255, 0, 0, "CD");
	Viewer.addLine(pcE, pcF, 255, 0, 0, "EF");
	//Viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 25, "AB");
	//Viewer.addCoordinateSystem(10);
	Viewer.setBackgroundColor(255, 255, 255);
	while (!Viewer.wasStopped())
	{
		Viewer.spinOnce(100);
	}
}

void GraspSet::Plot6DGrasp(const pcl::PointCloud<UserPointType>& cloud_filtered, const pcl::PointCloud<UserPointType>& cloud_fixture) {
	float sc = 20;  //sc1：坐标系长度，displace_x：抓取点上移距离
	double rgb_r = 0.545, rgb_g = 0.412, rgb_b = 0.0784, opactiy = 0.7, wide = 6;

	UserPointType pcA(displace_x, 0, 0), pcB(displace_x - 10, 0, 0),  //不能与param::displace_x混肴
		pcC(displace_x + param::finger_x, -param::finger_y, 0), pcD(displace_x + param::finger_x, param::finger_y, 0);
	UserPointType po(pcA.x + (wide / 2), 0, 0), px(po.x + sc, 0, 0), py(po.x, -sc, 0), pz(po.x, 0, -sc);

	pcl::visualization::PCLVisualizer Viewer;
	pcl::visualization::PointCloudColorHandlerCustom<UserPointType> color_handler1(cloud_filtered.makeShared(), 0, 255, 255);//0, 229, 238
	pcl::visualization::PointCloudColorHandlerCustom<UserPointType> color_handler2(cloud_fixture.makeShared(), 255, 0, 255);
	Viewer.addPointCloud(cloud_filtered.makeShared(), color_handler1, "cloud_filtered");
	Viewer.addPointCloud(cloud_fixture.makeShared(), color_handler2, "cloud_fixture");
	Viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud_filtered");
	Viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud_fixture");
	//Viewer.addSphere(po, 0.8, 255, 0, 0, "coordinate_origin ");

	/*添加坐标系*/
	//Viewer.addArrow(px, po, 1.0, 0.0, 0.0, false, "arrow_x");
	//Viewer.addArrow(py, po, 0.0, 1.0, 0.0, false, "arrow_y");
	//Viewer.addArrow(pz, po, 0.0, 0.0, 1.0, false, "arrow_z");

	/*添加抓取位姿*/
	Eigen::Vector3f center_A(pcA.x, pcA.y, pcA.z), center_B(pcB.x, pcB.y, pcB.z),
		center_C(pcC.x, pcC.y, pcC.z), center_D(pcD.x, pcD.y, pcD.z);
	Eigen::Quaternionf rotation(1, 0, 0, 0);
	
	Viewer.addCube(center_A, rotation, wide, 2 * param::finger_y + wide, 2 * param::finger_z, "cube1");
	Viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, rgb_r, rgb_g, rgb_b, "cube1");
	Viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, opactiy, "cube1");

	Viewer.addCube(center_B, rotation, 20 + wide, wide, 2 * param::finger_z, "cube2");
	Viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, rgb_r, rgb_g, rgb_b, "cube2");
	Viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, opactiy, "cube2");

	Viewer.addCube(center_C, rotation, 2 * param::finger_x, wide, 2 * param::finger_z, "cube3");
	Viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, rgb_r, rgb_g, rgb_b, "cube3");
	Viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, opactiy, "cube3");

	Viewer.addCube(center_D, rotation, 2 * param::finger_x, wide, 2 * param::finger_z, "cube4");
	Viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, rgb_r, rgb_g, rgb_b, "cube4");
	Viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, opactiy, "cube4");
	//Viewer.addCoordinateSystem(10);

	Viewer.setBackgroundColor(255, 255, 255);
	while (!Viewer.wasStopped())
	{
		Viewer.spinOnce(100);
	}
}


