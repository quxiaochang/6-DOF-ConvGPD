#include "cpp_py.h"

/*Python��ʼ��*/
bool Python::ModuleInit() {

	Py_Initialize(); //��ʼ��python������,���߱�����Ҫ�õ�python������

	import_array(); //����numpy_api

	dims[0] = 750;  //���ô���python�����С750 * 3
	dims[1] = 3;

	PyObject *pName, *pModule; //����python����
	pName = PyUnicode_FromString("estimator");  //ģ������:estimator.py
	pModule = PyImport_Import(pName);  //����ģ��
	pFunc = PyObject_GetAttrString(pModule, "Estimator"); //���뺯��
	pArgs = PyTuple_New(1);//����һ��Ԫ�飬����Ϊ1
	
	Py_DECREF(pName); //�ͷſռ�
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

/*����Python����*/
float Python::Estimator(float array[750][3]) {
	PyObject *mat = PyArray_SimpleNewFromData(2, dims, NPY_FLOAT, array);  //���ݽṹת��,C++������ת��Ϊpython��numpy
	PyTuple_SetItem(pArgs, 0, mat); //��pArgs�ĵ�һ����������Ϊmat
	PyObject *pReturn = PyObject_CallObject(pFunc, pArgs);  //����numpy����python����
	double out;
	out = PyFloat_AsDouble(pReturn);  //������5λС��
	float sorce = float(out);
	std::cout << "In C++ sorce: " << sorce << std::endl;

	Py_DECREF(mat); //�ͷſռ�
	Py_DECREF(pReturn);
	return sorce;
}

bool Python::Estimator(float array[750][3], std::vector<float> &scores) {
	scores.clear();

	//PyObject *mat = PyArray_SimpleNewFromData(2, dims, NPY_FLOAT, array);  //���ݽṹת��,C++������ת��Ϊpython��numpy
	PyTuple_SetItem(pArgs, 0, PyArray_SimpleNewFromData(2, dims, NPY_FLOAT, array)); //��pArgs�ĵ�һ����������Ϊmat
	pReturn = PyObject_CallObject(pFunc, pArgs);  //����numpy����python����

	double out;
	if (PyList_Check(pReturn)) {  //����Ƿ�ΪPython��List����
		Py_ssize_t SizeOfLise = PyList_Size(pReturn);  //����List�Ĵ�С3
		std::cout << "In C++ the len(List) = " << SizeOfLise << std::endl;

		for (Py_ssize_t i = 0; i < SizeOfLise; i++) {
			PyObject *ListItem = PyList_GetItem(pReturn, i);
			out = PyFloat_AsDouble(ListItem);  //����ת��

			scores.emplace_back(float(out));

			Py_DECREF(ListItem); //�ͷſռ�
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

/*����python���*/
void Python::PyClear() {
	Py_DECREF(pFunc);
	Py_DECREF(pArgs);
	Py_DECREF(pReturn);
	Py_Finalize();
}

