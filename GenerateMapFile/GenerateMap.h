#pragma once
#include<iostream>
#include<string>
#include<random>  
#include<time.h>  
#include<sstream>
#include<exception>
#include<fstream>
#include<direct.h>
class GenerateMap
{
private:
	int int_dim_x;//地图尺寸
	int int_dim_y;
	int** int_map;//地图内容 0障碍物 1可通行 -1待扩展
	std::string string_map_name;//写入文件的名字

	std::string GenerateMapName();//生成文件名
public:
	/*随机生成地图
	maptype 障碍物类型 0:点 1:图形 2:迷宫 
	density: 可通行率 既无障碍物点的比率*/
	bool RandGenerateMap(const int& maptype= 0, const double& density = 0.4);
public:
	GenerateMap();
	GenerateMap(const int& dimx, const int& dimy);
	~GenerateMap();
	bool WriteFile();//写入文件
	bool PrintConsole();//控制台输出
private:
	inline bool checkNodeInput(const int& x,const int& y);//检查坐标是否越界
	inline int visitNode(const int& x, const int& y);
	inline int visitDimX();
	inline int visitDimY();

};

