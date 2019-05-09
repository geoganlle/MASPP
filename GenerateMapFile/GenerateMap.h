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
	int int_dim_x;//��ͼ�ߴ�
	int int_dim_y;
	int** int_map;//��ͼ���� 0�ϰ��� 1��ͨ�� -1����չ
	std::string string_map_name;//д���ļ�������

	std::string GenerateMapName();//�����ļ���
public:
	/*������ɵ�ͼ
	maptype �ϰ������� 0:�� 1:ͼ�� 2:�Թ� 
	density: ��ͨ���� �����ϰ����ı���*/
	bool RandGenerateMap(const int& maptype= 0, const double& density = 0.4);
public:
	GenerateMap();
	GenerateMap(const int& dimx, const int& dimy);
	~GenerateMap();
	bool WriteFile();//д���ļ�
	bool PrintConsole();//����̨���
private:
	inline bool checkNodeInput(const int& x,const int& y);//��������Ƿ�Խ��
	inline int visitNode(const int& x, const int& y);
	inline int visitDimX();
	inline int visitDimY();

};

