#pragma once
#ifndef CONFIG_FILE_H_
#define CONFIG_FILE_H_


#include <fstream>
#include <iostream>
#include <map>
#include <string>
#include <sstream>
#include <typeinfo>
#include <Eigen/Core>
#include <vector>

class ConfigFile
{
public:

	ConfigFile(const std::string &fName);

	bool keyExists(const std::string &key) const;

	template <typename ValueType>
	ValueType getValueOfKey(const std::string &key, ValueType const &defaultValue) const
	{
		if (!keyExists(key)) {
			return defaultValue;
			std::cout << "Key not exist" << std::endl;
		}

		return string_to_T<ValueType>(contents.find(key)->second);
	}

	std::string getValueOfKeyAsString(const std::string &key, const std::string &defaultValue);

	template <typename T>
	std::string T_to_string(T const &val) const
	{
		std::ostringstream ostr;
		ostr << val;

		return ostr.str();
	}

	template <typename T>
	T string_to_T(std::string const &val) const
	{
		std::istringstream istr(val);
		T returnVal;
		if (!(istr >> returnVal))
			std::cout << "CFG: Not a valid " + (std::string)typeid(T).name() + " received!\n";

		return returnVal;
	}


private:

	void removeComment(std::string &line) const;

	bool onlyWhitespace(const std::string &line) const;

	bool validLine(const std::string &line) const;

	void extractKey(std::string &key, size_t const &sepPos, const std::string &line) const;

	void extractValue(std::string &value, size_t const &sepPos, const std::string &line) const;

	void extractContents(const std::string &line);

	void parseLine(const std::string &line, size_t const lineNo);

	void ExtractKeys();

	std::map<std::string, std::string> contents;
	std::string fName;
};

namespace param {
	extern bool plot_pc;
	extern bool plot_grasp;
	extern bool camera_test;
	extern bool serial_test;
	extern bool multi_detect;
	extern bool collect_train;
	extern bool dataset_test;
	extern bool force_closure_test;
	extern bool optimize_test;

	extern std::string pcd_path;
	extern std::string ply_path;
	extern std::string model_path;
	extern unsigned int set_sample;
	extern float finger_x;
	extern float finger_y;
	extern float finger_z;
	extern float finger_max;
	extern float displace_x;
	extern float displace_y;
	extern float displace_z;
	extern float filterlimits_z;
	extern float distance_threshold;

	extern float sample_leaf;
	extern float kdtree_rad;
	extern float hight_coe;
	extern float y_coe;
	extern std::vector<double> cluster_extract;
	extern std::vector<double> workspace;
	extern Eigen::Matrix4f wor2cam_trans;

	void SetParameter(const std::string &fName);
}

#endif /* CONFIG_FILE_H_ */

