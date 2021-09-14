#include "cnn.h"
#include "serial.h"
#include "arm.h"

int main() {
	param::SetParameter("params.cfg");
	CNN cnn(CovPoint, param::model_path);
	Conveyor conveyor(50);
	SCAN scanner;

	while (1) {
		if (!cnn.Run(scanner, conveyor))
			continue;
		//��ʱ5S,�ȴ���е�۸�λ
		//Sleep(10000);
	}
	cnn.PyClear(); //����python���
	std::cout << "Done! Press Enter to exit..." << std::endl;
	return 0;
}



//#include <Eigen/Core>
//#include <Eigen/Geometry>
//
//Eigen::Matrix3d eulerAnglesToRotationMatrix(Eigen::Vector3d& theta)
//{
//    Eigen::Matrix3d R_x;    // ������ת�����X����
//    R_x <<
//        1, 0, 0,
//        0, cos(theta[0]), -sin(theta[0]),
//        0, sin(theta[0]), cos(theta[0]);
//
//    Eigen::Matrix3d R_y;    // ������ת�����Y����
//    R_y <<
//        cos(theta[1]), 0, sin(theta[1]),
//        0, 1, 0,
//        -sin(theta[1]), 0, cos(theta[1]);
//
//    Eigen::Matrix3d R_z;    // ������ת�����Z����
//    R_z <<
//        cos(theta[2]), -sin(theta[2]), 0,
//        sin(theta[2]), cos(theta[2]), 0,
//        0, 0, 1;
//    Eigen::Matrix3d R = R_z * R_y * R_x;
//    return R;
//}
//
//int main() {
//    double DEG_TO_ARC = M_PI / 180, ARC_TO_DEC = 180 / M_PI;
//    double x_arc = 45 * DEG_TO_ARC;    // ��X�� -120,30,-60
//    double y_arc = -30 * DEG_TO_ARC;  // ��Y��
//    double z_arc = 24 * DEG_TO_ARC;      // ��Z��
//
//    cout << endl;
//    cout << "x_arc = " << x_arc << endl;
//    cout << "y_arc = " << y_arc << endl;
//    cout << "z_arc = " << z_arc << endl;
//
//    Eigen::Vector3d euler_angle(x_arc, y_arc, z_arc);
//    Eigen::Matrix3d rotation_matrix;
//    rotation_matrix = eulerAnglesToRotationMatrix(euler_angle);
//    std::cout << rotation_matrix << std::endl;
//
//    /*��ת������ŷ����*/
//    Eigen::Vector3d eulerAngle1 = rotation_matrix.eulerAngles(2, 1, 0); // ZYX˳��
//    cout << "x_1 y_1 z_1 = " << eulerAngle1[2] * ARC_TO_DEC << " " << eulerAngle1[1] * ARC_TO_DEC
//        << " " << eulerAngle1[0] * ARC_TO_DEC << " " <<eulerAngle1.transpose() << endl;
//
//    Eigen::Vector3d eulerAngle_(eulerAngle1[2], eulerAngle1[1], eulerAngle1[0]);
//    Eigen::Matrix3d rotation_matrix_ = eulerAnglesToRotationMatrix(eulerAngle_);
//    std::cout << rotation_matrix_ << std::endl;
//
//    /*��ת��������Ԫ��*/
//    Eigen::Quaterniond q(rotation_matrix);
//    std::cout << "x,y,z,w" << q.coeffs().transpose() << std::endl;
//
//    Eigen::Vector3d t1(0.3, 0.1, 0.1);
//    Eigen::Isometry3d T1(q);
//    std::cout << T1.matrix() << std::endl;
//    T1.pretranslate(t1);
//    std::cout << T1.matrix() << std::endl;
//
//}



