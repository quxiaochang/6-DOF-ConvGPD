#include "cpp_py.h"

/*Python初始化*/
bool Python::ModuleInit() {

	Py_Initialize(); //初始化python解释器,告诉编译器要用的python编译器

	import_array(); //加载numpy_api

	dims[0] = 750;  //设置传入python数组大小750 * 3
	dims[1] = 3;

	PyObject *pName, *pModule; //定义python对象
	pName = PyUnicode_FromString("estimator");  //模块名称:estimator.py
	pModule = PyImport_Import(pName);  //载入模块
	pFunc = PyObject_GetAttrString(pModule, "Estimator"); //载入函数
	pArgs = PyTuple_New(1);//创建一个元组，长度为1
	
	Py_DECREF(pName); //释放空间
	Py_DECREF(pModule);
	std::cout << "1";

	if (!pFunc || !PyCallable_Check(pFunc)) {
		std::cout << "error: pFunc" << std::endl;
		std::system("pause");
		return 0;
	}
	else {
		std::cout << "In C++: Success to link python and load module" << std::endl;
		return 1;
	}
		
}

/*调用Python函数*/
float Python::Estimator(float array[750][3]) {
	PyObject *mat = PyArray_SimpleNewFromData(2, dims, NPY_FLOAT, array);  //数据结构转换,C++的数组转换为python的numpy
	PyTuple_SetItem(pArgs, 0, mat); //将pArgs的第一个变量设置为mat
	PyObject *pReturn = PyObject_CallObject(pFunc, pArgs);  //参数numpy传入python函数
	double out;
	out = PyFloat_AsDouble(pReturn);  //保留了5位小数
	float sorce = float(out);
	std::cout << "In C++ sorce: " << sorce << std::endl;

	Py_DECREF(mat); //释放空间
	Py_DECREF(pReturn);
	return sorce;
}

bool Python::Estimator(float array[750][3], std::vector<float> &scores) {
	scores.clear();

	//PyObject *mat = PyArray_SimpleNewFromData(2, dims, NPY_FLOAT, array);  //数据结构转换,C++的数组转换为python的numpy
	PyTuple_SetItem(pArgs, 0, PyArray_SimpleNewFromData(2, dims, NPY_FLOAT, array)); //将pArgs的第一个变量设置为mat
	pReturn = PyObject_CallObject(pFunc, pArgs);  //参数numpy传入python函数

	double out;
	if (PyList_Check(pReturn)) {  //检测是否为Python的List对象
		Py_ssize_t SizeOfLise = PyList_Size(pReturn);  //返回List的大小3
		std::cout << "In C++ the len(List) = " << SizeOfLise << std::endl;

		for (Py_ssize_t i = 0; i < SizeOfLise; i++) {
			PyObject *ListItem = PyList_GetItem(pReturn, i);
			out = PyFloat_AsDouble(ListItem);  //数据转换

			scores.emplace_back(float(out));

			Py_DECREF(ListItem); //释放空间
		}
	}
	else {
		std::cout << "Not a List" << std::endl;
		return 0;
	}
	std::cout << "In C++ prediction list = " << scores[0] << " " << scores[1] 
		<< " " << scores[2] << std::endl;
	return !scores.empty();
}

/*销毁python相关*/
void Python::PyClear() {
	Py_DECREF(pFunc);
	Py_DECREF(pArgs);
	Py_DECREF(pReturn);
	Py_Finalize();
}

