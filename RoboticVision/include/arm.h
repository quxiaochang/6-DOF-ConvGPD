#pragma once
/************************************************************************/
/* ���ܣ���е�����˶�ѧ                                            */
/* ���ߣ���Т��                                                    */
/* �汾��v1.0                                                      */
/************************************************************************/


#ifndef ARM_H_
#define ARM_H_
#include "grasp.h"

class Arm {
public:
	Arm() = default;
	Arm(float OB_, float BC_, float CE_, float EG_) :OB(OB_), BC(BC_), CE(CE_), EG(EG_) {}

	void ManiInverse(const GraspSet& graspset, std::vector<float>& theta_steer);

private:
	float OB, BC, CE, EG;
	
};


#endif

