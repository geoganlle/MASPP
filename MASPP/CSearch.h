/*
program file No.5
������

Written by Geoganlle Goo
**/
#pragma once
#include "CGridMap.h"
#include "CState.h"
#include "CDistance.h"
#include <vector>
#include <unordered_map>

constexpr auto EXPLIM = 2000000;	// ���ڵ㴦����;�ο�:���intΪ2147483647

/*
���ÿ��������Ľڵ�ṹ��
��װ��A*�㷨������ʽ��ֵf(n)
��state�����˷�װ�����������߼���֯�ṹ
->node(0)<-node(1)<-node(2)  
A->B��ʾA�ĸ�ָ��ָ��B
**/
struct stNode_Search {
	int f;//�ɱ���ֵ f(n) = g(n) + h(n)
	short agentid_short;//Ҫ�ƶ��������id
	short dir_from_parent;//�Ӹ��ڵ���ʱ�ķ���
	CState* state_CStatep;//�ýڵ��״̬
	stNode_Search* parent_stNodep;//���ڵ�
	bool operator<(const stNode_Search& stNode_input) const { return stNode_input.f < f; };
};

class CSearch
{
//variate:
	int n_agentnumber_int;//�����������
	int	expansions_node_number_int;	//��չ�ڵ������
	int	cost_path_int;//·���ĳɱ�

	CGridMap* gridmap;//��ͼ
	CDistance* distance_CDistance;//�����
	stPoint* goal_stPointp;	// Ŀ��״̬��
	stPoint* init_stPointp;	// ��ʼ״̬��

	stNode_Search* last_stNode;//����ҵ�Ŀ��ڵ�������Ϊ��ǰ�ڵ�

	std::unordered_map<int,stAgentPosition>* CAT;//��ײ����� Collision avoidance table
	/*
	int timestep ��·�������ϵĿ��������������������λ�ýṹ��
	stAgentPosition �������λ��
	*/

	std::priority_queue<stNode_Search*> open_priority_queue;//����ɵ������� ���ȶ��� �󶥶�
	std::vector<stNode_Search*> closed_vector;//����ɵ�������

//function:

	stNode_Search* generate_childnode(stNode_Search* p, int dir);//����һ���ڵ���ӽڵ�
	bool is_goal_stNode(stNode_Search* node);//���Ŀ��ڵ��Ƿ�Ϸ�
	std::vector<int>* getpath(stNode_Search* n);	//����Ӹ��ڵ㵽n�ڵ��·��(����)
	int* get_hashPoint_of_path(int agent, const std::vector<int>& tr);//�õ�·����hash������

public:
	int	expand();//ѡ��һ��f(n)��С�ĵ�	����ֵ˵����1Ŀ��ڵ� 2����ڵ�������
	int	get_num_expansions_int();//������չ�ڵ���
	int get_cost_int();	//�����ɱ�

	std::vector<int>* path(bool print);	//��Ŀ���м����ҵ���·��

	CSearch(int n, stPoint* init, stPoint* goal, CGridMap* g, CDistance* d = NULL,
		std::unordered_map<int,stAgentPosition>* cat = NULL);
	~CSearch();
};

inline
int CSearch::get_num_expansions_int() { 
	return expansions_node_number_int; 
}

inline int CSearch::get_cost_int()
{
	return cost_path_int;
}
