#include "config_file.h"

// function to read in a double array from a single line of a configuration file
std::vector<double> stringToDouble(const std::string& str)
{
	std::vector<double> values;
	std::stringstream ss(str);
	double v;

	while (ss >> v)
	{
		values.push_back(v);
		if (ss.peek() == ' ')
		{
			ss.ignore();
		}
	}

	return values;
}

void ConfigFile::removeComment(std::string &line) const
{
	if (line.find('#') != line.npos)
		line.erase(line.find('#'));
}


bool ConfigFile::onlyWhitespace(const std::string &line) const
{
	return (line.find_first_not_of(' ') == line.npos);
}


bool ConfigFile::validLine(const std::string &line) const
{
	std::string temp = line;
	temp.erase(0, temp.find_first_not_of("\t "));
	if (temp[0] == '=')
		return false;

	for (size_t i = temp.find('=') + 1; i < temp.length(); i++)
		if (temp[i] != ' ')
			return true;

	return false;
}


void ConfigFile::extractKey(std::string &key, size_t const &sepPos, const std::string &line) const
{
	key = line.substr(0, sepPos);
	if (key.find('\t') != line.npos || key.find(' ') != line.npos)
		key.erase(key.find_first_of("\t "));
}


void ConfigFile::extractValue(std::string &value, size_t const &sepPos, const std::string &line) const
{
	value = line.substr(sepPos + 1);
	value.erase(0, value.find_first_not_of("\t "));
	value.erase(value.find_last_not_of("\t ") + 1);
}


void ConfigFile::extractContents(const std::string &line)
{
	std::string temp = line;
	temp.erase(0, temp.find_first_not_of("\t "));
	size_t sepPos = temp.find('=');

	std::string key, value;
	extractKey(key, sepPos, temp);
	extractValue(value, sepPos, temp);

	if (!keyExists(key))
		contents.insert(std::pair<std::string, std::string>(key, value));
	else
		std::cout << "CFG: Can only have unique key names!\n";
}


void ConfigFile::parseLine(const std::string &line, size_t const lineNo)
{
	if (line.find('=') == line.npos)
		std::cout << "CFG: Couldn't find separator on line: " + T_to_string(lineNo) + "\n";

	if (!validLine(line))
		std::cout << "CFG: Bad format for line: " + T_to_string(lineNo) + "\n";

	extractContents(line);
}


void ConfigFile::ExtractKeys()
{
	std::ifstream file;
	file.open(fName.c_str());
	if (!file)
		std::cout << "Config file " + fName + " could not be found!\n";

	std::string line;
	size_t lineNo = 0;
	while (std::getline(file, line))
	{
		lineNo++;
		std::string temp = line;

		if (temp.empty())
			continue;

		removeComment(temp);
		if (onlyWhitespace(temp))
			continue;

		parseLine(temp, lineNo);
	}

	file.close();
}


ConfigFile::ConfigFile(const std::string &fName)
{
	this->fName = fName;
	ExtractKeys();
}


bool ConfigFile::keyExists(const std::string &key) const
{
	return contents.find(key) != contents.end();
}


std::string ConfigFile::getValueOfKeyAsString(const std::string &key, const std::string &defaultValue)
{
	if (!keyExists(key))
		return defaultValue;

	return contents.find(key)->second;
}

namespace param {
	bool plot_pc;
	bool plot_grasp;
	bool camera_test;
	bool serial_test;
	bool multi_detect;
	bool collect_train;
	bool dataset_test;
	bool force_closure_test;
	bool optimize_test;
	std::string pcd_path;
	std::string ply_path;
	std::string model_path;
	unsigned int set_sample;
	float finger_x;
	float finger_y;
	float finger_z;
	float finger_max;
	float displace_x;
	float displace_y;
	float displace_z;
	
	float filterlimits_z;
	float distance_threshold;
	float sample_leaf;
	float kdtree_rad;
	float hight_coe;
	float y_coe;
	std::vector<double> cluster_extract;
	std::vector<double> workspace;
	Eigen::Matrix4f wor2cam_trans;
}

void param::SetParameter(const std::string &fName) {
	ConfigFile config_file(fName);
	std::string workspace_str = config_file.getValueOfKeyAsString("workspace", "");
	std::string wor2cam_str = config_file.getValueOfKeyAsString("wor2cam_mat", "");
	std::string cluster_extract_str = config_file.getValueOfKeyAsString("cluster_extraction", "");
	pcd_path = config_file.getValueOfKeyAsString("pcd_path", "");
	ply_path = config_file.getValueOfKeyAsString("ply_path", "");
	model_path = config_file.getValueOfKeyAsString("model_path", "");
	plot_pc = config_file.getValueOfKey<bool>("plot_pc", true);
	plot_grasp = config_file.getValueOfKey<bool>("plot_grasp", true);
	camera_test = config_file.getValueOfKey<bool>("camera_test", false);
	serial_test = config_file.getValueOfKey<bool>("serial_test", false);
	multi_detect = config_file.getValueOfKey<bool>("multi_detect", false);
	dataset_test = config_file.getValueOfKey<bool>("dataset_test", false);
	collect_train = config_file.getValueOfKey<bool>("collect_train", false);
	force_closure_test = config_file.getValueOfKey<bool>("force_closure_test", false);

	finger_x = config_file.getValueOfKey<float>("finger_x", 0);
	finger_y = config_file.getValueOfKey<float>("finger_y", 0);
	finger_z = config_file.getValueOfKey<float>("finger_z", 0);
	finger_max = config_file.getValueOfKey<float>("finger_max", 0);
	displace_x = config_file.getValueOfKey<float>("displace_x", 0);
	displace_y = config_file.getValueOfKey<float>("displace_y", 0);
	displace_z = config_file.getValueOfKey<float>("displace_z", 0);

	optimize_test = config_file.getValueOfKey<bool>("optimize_test", 0);
	filterlimits_z = config_file.getValueOfKey<float>("filterlimits_z", 0);
	distance_threshold = config_file.getValueOfKey<float>("distance_threshold", 0);

	sample_leaf = config_file.getValueOfKey<float>("sample_leaf", 0);
	kdtree_rad = config_file.getValueOfKey<float>("kdtree_rad", 0);
	hight_coe = config_file.getValueOfKey<float>("hight_coe", 0);
	y_coe = config_file.getValueOfKey<float>("y_coe", 0);
	set_sample = config_file.getValueOfKey<unsigned int>("set_sample", 0);

	workspace = stringToDouble(workspace_str);
	cluster_extract = stringToDouble(cluster_extract_str);
	std::vector<double> trans = stringToDouble(wor2cam_str);
	wor2cam_trans << trans[0], trans[1], trans[2], trans[3],
		trans[4], trans[5], trans[6], trans[7],
		trans[8], trans[9], trans[10], trans[11],
		0, 0, 0, 1;

	std::cout << "pcd_path: " << pcd_path << std::endl;
	std::cout << "ply_path: " << ply_path << std::endl;
	std::cout << "model_path: " << model_path << std::endl;
	std::cout << "workspace: " << workspace_str << std::endl;
	std::cout << "cluster_extract: " << cluster_extract_str << std::endl;
	std::cout << "plot_pc: " << plot_pc << std::endl;
	std::cout << "plot_grasp: " << plot_grasp << std::endl;
	std::cout << "camera_test: " << camera_test << std::endl;
	std::cout << "serial_test: " << serial_test << std::endl;
	std::cout << "dataset_test: " << dataset_test << std::endl;
	std::cout << "multi_detect: " << multi_detect << std::endl;
	std::cout << "collect_train: " << collect_train << std::endl;
	std::cout << "force_closure_test: " << force_closure_test << std::endl;
	std::cout << "finger_x: " << finger_x << std::endl;
	std::cout << "finger_y: " << finger_y << std::endl;
	std::cout << "finger_z: " << finger_z << std::endl;
	std::cout << "finger_max: " << finger_max << std::endl;
	std::cout << "displace_x: " << displace_x << std::endl;
	std::cout << "displace_y: " << displace_y << std::endl;
	std::cout << "displace_z: " << displace_z << std::endl;

	std::cout << "optimize_test: " << optimize_test << std::endl;
	std::cout << "filterlimits_z: " << filterlimits_z << std::endl;
	std::cout << "distance_threshold: " << distance_threshold << std::endl;
	std::cout << "sample_leaf: " << sample_leaf << std::endl;
	std::cout << "set_sample: " << set_sample << std::endl;
	std::cout << "kdtree_rad: " << kdtree_rad << std::endl;
	std::cout << "hight_coe: " << hight_coe << std::endl;
	std::cout << "y_coe: " << y_coe << std::endl;
	std::cout << "wor2cam_trans:\n" << wor2cam_trans << std::endl;
}
