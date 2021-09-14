#pragma once
/***************************************************************************************/
/* 功能：链接到python(pytorch与Libtorch版本必须一样),在模型文件pth转换不了pt时使用     */
/* 作者：瞿孝昌                                                                        */
/* 版本：v1.0                                                                          */
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
	PyObject* pFunc, * pArgs, * pReturn; //定义python对象:函数、参数
	npy_intp dims[2];  //参数数组大小
};


#endif


