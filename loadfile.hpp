#ifndef LOAD_FILE_
#define LOAD_FILE_

#include <iostream>
#include <ostream>
#include <string>
#include <vector>
#include <pcl/point_cloud.h>	// 这个注释掉也可以用
#include <pcl/point_types.h>	// 这个注释掉也可以用


using namespace std;


void fileToCloud(string filenameIn, pcl::PointCloud<pcl::PointXYZI>::Ptr cloudOut) {
    string postfix = filenameIn.substr(filenameIn.length() - 3, filenameIn.length());
		
	if (postfix == "txt") {
		ifstream inputfile;
		inputfile.open(filenameIn);	// 打开模式ios_base::in，也就是默认模式，可以省略
		
		if (!inputfile) {
			cerr << "Open input file error!" << endl;
			exit(-1);
		} else {
			cout << "Open file \"" << filenameIn << "\"" << endl;
		}

		// 如何能根据模板参数直接判断是否有Intensity信息呢？能否做一个函数重载，不要模板参数了？
		float x, y, z;
		int intensity;
		int scan_ID;
		vector<float> range_xy_, angle_;
		while(inputfile.good()) {
			inputfile >> x >> y >> z >> intensity;
			// 直接在生成点云是就给它滤波了
			pcl::PointXYZI p;
			p.x = x;
			p.y = y;
			p.z = z;
			p.intensity = intensity;
			cloudOut->points.push_back(p);
		}
		
		cloudOut->height = 1;
		cloudOut->width = cloudOut->points.size();
	}
	else if (postfix == "bin") {
		ifstream inputfile;
		pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(
			new pcl::PointCloud<pcl::PointXYZI>);

		inputfile.open(filenameIn, ios::binary);
		if (!inputfile) {
			cerr << "Open input file error!" << endl;
			exit(-1);
		} else {
			cout << "Open file \"" << filenameIn << "\"" << endl;
		}
		
		inputfile.seekg(0, ios::beg); // param:文件偏移量；从哪里计算偏移量

		for (int i = 0; inputfile.good() && !inputfile.eof(); i++) {
			pcl::PointXYZI p;
			inputfile.read((char *) &p.x, 3 * sizeof(float));
			inputfile.read((char *) &p.intensity, sizeof(float));	// 这里的LIDar的intensity用的是浮点数，0-1
			cloudOut->points.push_back(p);
		}
		cloudOut->height = 1;
		cloudOut->width = cloudOut->points.size();
	}

}

#endif