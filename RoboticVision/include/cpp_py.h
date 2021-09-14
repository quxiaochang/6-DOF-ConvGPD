#pragma once
/***************************************************************************************/
/* ���ܣ����ӵ�python(pytorch��Libtorch�汾����һ��),��ģ���ļ�pthת������ptʱʹ��     */
/* ���ߣ���Т��                                                                        */
/* �汾��v1.0                                                                          */
/***************************************************************************************/

#ifndef PYTHON_H__
#define PYTHON_H__

/*system*/
#include <iostream>
#include <vector>
/*user*/

/*Python*/
#include <Python.h>
#include <numpy/arrayobject.h>

class Python {
public:
	Python() = default;

	bool ModuleInit();
	float Estimator(float array[750][3]);
	bool Estimator(float array[750][3], std::vector<float> &scores);
	void PyClear();

private:
	PyObject* pFunc, * pArgs, * pReturn; //����python����:����������
	npy_intp dims[2];  //���������С
};


#endif


