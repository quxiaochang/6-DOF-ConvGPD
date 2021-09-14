#include "arm.h"

void Arm::ManiInverse(const GraspSet& graspset, std::vector<float>& theta_steer) {
	theta_steer.clear();

	float theta[6];
	float Xe(0), Ye(0), Ze(0),
		Xg(graspset.grasp_point.x), Yg(graspset.grasp_point.y), Zg(graspset.grasp_point.z);

	Eigen::Vector3f pointe_cam, pointe_wor;
	pointe_cam << (Xg - EG * graspset.fix_axis_x(0)), (Yg - EG * graspset.fix_axis_x(1)),
		(Zg - EG * graspset.fix_axis_x(2));
	Eigen::Affine3f wor2cam_aff(param::wor2cam_trans);
	pcl::transformPoint(pointe_cam, pointe_wor, wor2cam_aff);
	std::cout << "point e in camera:" << pointe_cam << std::endl;
	std::cout << "point e in world:" << pointe_wor << std::endl;
	Xe = pointe_wor(0);
	Ye = pointe_wor(1);
	Ze = pointe_wor(2);

	theta[0] = atan2(-Ye, Xe);  //弧度
	std::cout << theta[0] * 180 / M_PI << std::endl;
	float temp_g = Xe / cos(theta[0]);
	float temp_h = Ze - OB;

	theta[2] = asin((pow(temp_g, 2) + pow(temp_h, 2) - pow(BC, 2) - pow(CE, 2)) / (2.0 * BC * CE));  //自变量超出[-1,1]，表示超出机械臂自身长度的限制,-nan(ind)

	float temp_d = CE * cos(theta[2]);
	float temp_f = CE * sin(theta[2]) + BC;

	theta[1] = atan2((temp_g * temp_f - temp_h * temp_d), (temp_g * temp_d + temp_h * temp_f));

	
	Eigen::Matrix3f rotate_01, rotate_12, rotate_23, rotate_03, rotate_36, rotate_wor2fix;
	rotate_01 << cos(theta[0]), sin(theta[0]), 0,
		-sin(theta[0]), cos(theta[0]), 0,
		0, 0, 1;
	rotate_12 << cos(theta[1]), 0, sin(theta[1]),
		0, 1, 0,
		-sin(theta[1]), 0, cos(theta[1]);
	rotate_23 << cos(theta[2]), 0, -sin(theta[2]),
		0, 1, 0,
		sin(theta[2]), 0, cos(theta[2]);
	rotate_03 = rotate_01 * rotate_12 * rotate_23;

	/*世界坐标系转化为夹爪坐标系*/
	rotate_wor2fix = (param::wor2cam_trans * graspset.fix2cam_mat.inverse()).block(0, 0, 3, 3);
	rotate_36 = rotate_03.inverse() * rotate_wor2fix;
	std::cout << "mat_wor2fix: " << (param::wor2cam_trans * graspset.fix2cam_mat.inverse()) << std::endl;
	std::cout << "rotate_wor2fix: " << rotate_wor2fix << std::endl;
	std::cout << "rotate_36: " << rotate_36 << std::endl;

	theta[4] = atan2(sqrt(pow(rotate_36(0, 1), 2) + pow(rotate_36(0, 2), 2)), rotate_36(0, 0));
	if (theta[4] != 0) {
		theta[3] = atan2(-rotate_36(1, 0), -rotate_36(2, 0));  //加上负号是为了防止分子分母为0
		theta[5] = atan2(-rotate_36(0, 1), rotate_36(0, 2));
	}
	else {  //存疑
		theta[3] = 0;  //优先使4号舵机为0
		theta[5] = atan2(rotate_36(1, 2), rotate_36(1, 1));
	}

	
	if ((theta[5] * 180 / M_PI) < -90)
		theta[5] += M_PI;
	if ((theta[5] * 180 / M_PI) > 90)
		theta[5] -= M_PI;
	theta[5] -= M_PI / 16;

	std::cout << "theta: ";
	for (int i = 0; i < 6; i++) {
		float theta_ang = theta[i] * 180 / M_PI;
		if (abs(theta_ang) < 1)  //过小不能正常传给机械臂
			theta_ang = 0;
		theta_steer.emplace_back(theta_ang);  //角度制
		std::cout << theta_steer[i] << " ";
	}
	std::cout << std::endl;
}