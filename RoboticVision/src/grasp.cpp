#pragma warning(disable:4996)
#include "grasp.h"

GraspSet::GraspSet(const UserPointType& sample_point_, const UserPointType& centroid_point_) {
	sample_point = sample_point_;
	centroid_point = centroid_point_;
	displace_x = param::displace_x;
}

/*����о�X��������ץȡ��*/
bool GraspSet::CalcuAxisXGrasp(Eigen::Vector3f& normal_vector) {
	/*��������, ����ָ���ڲ�*/
	float vector_dot = 0;
	if (param::dataset_test)
		vector_dot = normal_vector.dot(centroid_point.getVector3fMap() - sample_point.getVector3fMap());  //������ˣ�����|a||b|cos��
	else {
		Eigen::Vector3f view_vector(sample_point.x, sample_point.y, (sample_point.z - 500)); //LMI���У׼֮���ӽǵ�Ĵ���λ��Ϊ(0,0,500)
		vector_dot = normal_vector.dot(view_vector);
	}
	std::cout << "��˻�ȡ|a||b|cos��:" << vector_dot << std::endl;

	if (vector_dot < 0) {
		normal_vector *= -1;
		std::cout << "����֮�����������ve(3x3):\n" << normal_vector << std::endl;
	}

	fix_axis_x = normal_vector.normalized();  //����������һ��
	float x_norm = fix_axis_x.norm();
	std::cout << "fix_axis_x:" << fix_axis_x << std::endl;
	std::cout << "fix_axis_x.norm:" << x_norm << std::endl;

	/*��ʼ��ץȡ�㣬ץȡ��������������(����ϵԭ��)ƫ��displace_x*/
	grasp_point.x = displace_x * fix_axis_x(0) + sample_point.x;
	grasp_point.y = displace_x * fix_axis_x(1) + sample_point.y;
	grasp_point.z = displace_x * fix_axis_x(2) + sample_point.z;

	return (!fix_axis_x.isZero());
}

/*����о�YZ�������ͱ任����*/
bool GraspSet::CalcuMatAxisYZ(Eigen::Vector3f& princom_vector) {
	princom_vector.normalize();  
	if (fix_axis_x == princom_vector || fix_axis_x == (-princom_vector)) {
		std::cout << "fix_axis_x and princom_vector are in the same direction!" << std::endl;
		return false;
	}
	else {
		fix_axis_y = fix_axis_x.cross(princom_vector); //ab������ˣ�����ab�Ĵ�ֱ����
		fix_axis_y.normalize();  //��һ��

		float y_norm = fix_axis_y.norm();
		std::cout << "fix_axis_y:" << fix_axis_y << std::endl;
		std::cout << "fix_axis_y.norm:" << y_norm << std::endl;

		fix_axis_z = fix_axis_x.cross(fix_axis_y); //������ˣ�˳���ܻ���
		fix_axis_z.normalize();
		float z_norm = fix_axis_z.norm();
		std::cout << "fix_axis_z:" << fix_axis_z << std::endl;
		std::cout << "fix_axis_z.norm:" << z_norm << std::endl;

		CalcuTranMatrix();  

		return(!fix_axis_y.isZero() && !fix_axis_z.isZero());
	}
}

/*����SVD�������о�����ϵ���������ϵ�ı任����*/
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

	/*�����������ϵ�ĵ�*/
	cloud_in->points[0] = (UserPointType(sample_point.x, sample_point.y, sample_point.z));                                                 //ԭ��
	cloud_in->points[1] = (UserPointType(sample_point.x + fix_axis_x(0), sample_point.y + fix_axis_x(1), sample_point.z + fix_axis_x(2))); //X��
	cloud_in->points[2] = (UserPointType(sample_point.x + fix_axis_y(0), sample_point.y + fix_axis_y(1), sample_point.z + fix_axis_y(2))); //Y��
	cloud_in->points[3] = (UserPointType(sample_point.x + fix_axis_z(0), sample_point.y + fix_axis_z(1), sample_point.z + fix_axis_z(2))); //Z��

	/*�����צ����ϵ�Ķ�Ӧ��*/
	cloud_out->points[0] = (UserPointType(0, 0, 0));
	cloud_out->points[1] = (UserPointType(1, 0, 0));
	cloud_out->points[2] = (UserPointType(0, 1, 0));
	cloud_out->points[3] = (UserPointType(0, 0, 1));

	//����SVD�������任����  
	pcl::registration::TransformationEstimationSVD<UserPointType, UserPointType> TESVD;
	//pcl::registration::TransformationEstimationSVD<UserPointType, UserPointType>::Matrix4 transformation2;
	
	TESVD.estimateRigidTransformation(*cloud_in, *cloud_out, fix2cam_mat);

	auto end_time = std::chrono::steady_clock::now();
	auto total_span = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
	std::cout << "CalcuTranMatrix runtime:" << total_span.count() << " ms" << std::endl;

	//����任������Ϣ  
	std::cout << "The Estimated Rotation and translation matrices (using getTransformation function) are : \n" << fix2cam_mat << std::endl ;
}

bool GraspSet::Algorithm1(Eigen::Vector3f& normal_vector, Eigen::Vector3f& princom_vector) {
	if (!CalcuAxisXGrasp(normal_vector))  //��ȡ�о�X��������ץȡ��
		return false;

	OptimizeTest(param::optimize_test, princom_vector); //�Ż����

	if (!CalcuMatAxisYZ(princom_vector))  //��ȡ�о�YZ��������任����
		return false;

	return true;
}


bool GraspSet::Algorithm2(const Eigen::Matrix3f& curvature_axis) {
	Eigen::Vector3f curmin_vector = curvature_axis.col(0);
	Eigen::Vector3f curmax_vector = curvature_axis.col(1);
	Eigen::Vector3f normal_vector = curvature_axis.col(2);

	displace_x = param::displace_x;  //Algorithm1�Ż�ʱ�޸���Ĭ�ϵ�ֵ
	if (!CalcuAxisXGrasp(curmax_vector))  //curmax_vector��Ϊ�о�X�ᣬ����curmin_vector
		return false;

	if (!CalcuMatAxisYZ(normal_vector))  //curmin_vector����ȡ�о�YZ��������任����,Y��ȷ��Ϊnormal_vector
		return false;

	return true;
}

void GraspSet::FiltFixturePC(const pcl::PointCloud<UserPointType>& cloud_filtered, pcl::PointCloud<UserPointType>& cloud_fixture, size_t& pc_arm) {
	cloud_fixture.clear();

	pcl::PointCloud<UserPointType>::Ptr cloud_transform(new pcl::PointCloud<UserPointType>); //ת�����о�����ϵ														
	pcl::transformPointCloud(cloud_filtered, *cloud_transform, fix2cam_mat); //cloud_transform = mat* cloud_filtered

	pcl::PointCloud<UserPointType>::Ptr cloud_arm(new pcl::PointCloud<UserPointType>);
	pcl::PassThrough<UserPointType> pass;
	/*ת��֮��ֱͨ�˲�*/
	pass.setInputCloud(cloud_transform);
	pass.setFilterFieldName("z");
	pass.setFilterLimits(-param::finger_z, param::finger_z);  //��λ��mm
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
	FiltFixturePC(cloud_filtered, cloud_fixture, pc_arm);//cloud_filtered������Ԥ����ProcessPC֮��ĵ���

	if (pc.SegmentPC(cloud_fixture) > 1 || grasp_point.z < (2 * param::finger_x)) { //��������ײ����true!ClosureCollisionDetect(cloud_fixture)
		OptimizeGraspDisplace(pc);//λ���Ż�graspset���޸�displace_x��grasp_point����������
		FiltFixturePC(cloud_filtered, cloud_fixture, pc_arm);
	}
	if (rotate_flag && PreScreenCand(cloud_fixture, pc_arm)) {
		OptimizeGraspRotate(cloud_fixture, pc);  //��ת�Ż�graspset���仯��λ���Ż��෴
		FiltFixturePC(cloud_filtered, cloud_fixture, pc_arm);
	}

	return PreScreenCand(cloud_fixture, pc_arm);
}

void GraspSet::OptimizeGraspDisplace(ProcessCloud& pc) {
	//if (pc.SegmentPC(cloud_fixture) > 1) {
	//	std::cout << "There is more than one pc in cloud_fixture" << std::endl;
	std::cout << "Start to optimize grasp displace!" << std::endl;
		float scl = 10;//��x�ᷴ����λ�ƣ���sample_point��ĩ��10mm
		displace_x = scl - 2 * param::finger_x; //�޸�displace_x
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

	Eigen::Matrix4f ofix2fix_rotate = Eigen::Matrix4f::Identity();   //��ת����
	
	float theta_left(0), theta_right(0), theta_x(0); //��ϵ�ֱ����y��ļн�,������,[0,pi]
	if (pc.CalAngleY(cloud_fixture, theta_left, theta_right)) {
		theta_x = (theta_left - M_PI + theta_right) / 2; //���ַ���ԭ�о�����ϵ��x����ת�ĽǶ�

		if (param::force_closure_test) {
			std::cout << "a1 before optimization: " << abs(theta_left - M_PI_2) / M_PI * 180 << std::endl;
			std::cout << "a2 before optimization:: " << abs(theta_right - M_PI_2) / M_PI * 180 << std::endl;
		}
		ofix2fix_rotate << 1, 0, 0, 0,
			0, cos(theta_x), sin(theta_x), 0,
			0, -sin(theta_x), cos(theta_x), 0,
			0, 0, 0, 1;
		fix2cam_mat = ofix2fix_rotate * fix2cam_mat;  //�Ż�֮��ļо�����ϵת����֮ǰ�о�����ϵ����ת�����������ϵ

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

/*������о߷��������ײ��⣬��������ײ����true*/
bool GraspSet::ClosureCollisionDetect(const pcl::PointCloud<UserPointType>& cloud_fixture) {
	/*��ȡ��ȡ���*/
	UserPointType min_p1, max_p1;  //��ײ�������
	pcl::getMinMax3D(cloud_fixture, min_p1, max_p1);

	std::cout <<"min_p1: "<< min_p1 << max_p1 << std::endl;
	float coll_coeff = 0.9;  //����Ϊ1�������жϲ�׼

	return (min_p1.x > (coll_coeff * displace_x)) &&
		(min_p1.y > (coll_coeff * (-param::finger_y))) &&
		(max_p1.y < (coll_coeff* param::finger_y));
}

/*�������е����ײ��⣬��������ײ����true*/
bool GraspSet::ArmCollisionDetect(const size_t& cloud_arm) {
	return  cloud_arm < 10;
}

/*�жϼо��ڲ������Ƿ��㹻�࣬�ﵽ��ֵ����true*/
bool GraspSet::PCNumberDetect(const pcl::PointCloud<UserPointType>& cloud_fixture) {
	int min_pc_fix(0);
	if (param::dataset_test)  //YCB�����ݼ��������ϡ�裬���趨��ֵСһ��
		min_pc_fix = 10;
	else
		min_pc_fix = 50;

	return cloud_fixture.size() > min_pc_fix;
}

/*�ж�ץȡλ�˵ĸ߶������������ļн�*/
bool GraspSet::PostureDetect() {
	Eigen::Vector3f gravity_vector(0, 0, -1); //������������
	double cos_val = gravity_vector.dot(fix_axis_x) / (gravity_vector.norm() * fix_axis_x.norm()); //�Ƕ�cosֵ
	double angle = acos(cos_val) * 180 / M_PI;     //����[0,pi]֮��ļ���������λΪ����
	std::cout << "The angle between the X axis and the gravity vector: " <<angle << std::endl;
	return angle < 90 && grasp_point.z > (2 * param::finger_x);  //ץȡλ�������������ļнǡ��߶��ж�
}

/*ץȡλ�˳���ɸѡ*/
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
	float sc1 = 20;  //sc1������ϵ���ȣ�displace_x��ץȡ�����ƾ���
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
	Eigen::Matrix4f tm_inv = Eigen::Matrix4f::Identity();  //��ʼ��Ϊ��λ����
	tm_inv = fix2cam_mat.inverse();
	pcl::transformPointCloud(cloud_fixture, *cloud_camera, fix2cam_mat.inverse());  //���о�����ϵ����ת����ȥ

	pcl::visualization::PCLVisualizer Viewer;
	pcl::visualization::PointCloudColorHandlerCustom<UserPointType> color_handler1(cloud_filtered.makeShared(), 0, 255, 255);//0, 229, 238
	pcl::visualization::PointCloudColorHandlerCustom<UserPointType> color_handler2(cloud_camera, 255, 0, 255);
	Viewer.addPointCloud(cloud_filtered.makeShared(), color_handler1, "cloud_filtered");
	Viewer.addPointCloud(cloud_camera, color_handler2, "cloud_camera");
	Viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud_filtered");
	Viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud_camera");
	Viewer.addSphere(cp, 0.8, 255, 0, 0, "sample_point");

	/*�������ϵ*/
	Viewer.addArrow(pcX, cp, 1.0, 0.0, 0.0, false, "arrow_x");
	Viewer.addArrow(pcY, cp, 0.0, 1.0, 0.0, false, "arrow_y");
	Viewer.addArrow(pcZ, cp, 0.0, 0.0, 1.0, false, "arrow_z");

	/*���ץȡλ��*/
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
	float sc = 20;  //sc1������ϵ���ȣ�displace_x��ץȡ�����ƾ���
	double rgb_r = 0.545, rgb_g = 0.412, rgb_b = 0.0784, opactiy = 0.7, wide = 6;

	UserPointType pcA(displace_x, 0, 0), pcB(displace_x - 10, 0, 0),  //������param::displace_x����
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

	/*�������ϵ*/
	//Viewer.addArrow(px, po, 1.0, 0.0, 0.0, false, "arrow_x");
	//Viewer.addArrow(py, po, 0.0, 1.0, 0.0, false, "arrow_y");
	//Viewer.addArrow(pz, po, 0.0, 0.0, 1.0, false, "arrow_z");

	/*���ץȡλ��*/
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


