#pragma once
#include "CGridMap.h"
#include "CState.h"
#include "CDistance.h"
#include <vector>
#include <unordered_map>
#define EXPLIM 20000000	// ���ڵ㴦����
//#define MAXINT 2147483647 //���intΪ2147483647
struct stNode {
	int f;//�ɱ���ֵ f(n) = g(n) + h(n)
	short agentid_short;//Ҫ�ƶ��������id
	short dir_from_parent;//�Ӹ��ڵ���ʱ�ķ���
	CState* state_CStatep;//�ýڵ��״̬
	stNode* parent_stNodep;//��һ���ڵ�
	bool operator<(const stNode& stNode_input) const { return stNode_input.f < f; };
};

////////////////////////////////////////////////////////////////
struct stNode_t {
	stNode* n;
	stNode_t(stNode* ptr) : n(ptr) {};
	bool operator<(const stNode_t& stNode_t_input) const {
		return n->operator<(*(stNode_t_input.n));
	}
};

class CSearch
{
	int n_agentnumber_int;//�����������
	int	expansions_node_number_int;	//��չ�ڵ������
	int	cost_path_int;//·���ĳɱ�
	CGridMap* gridmap;
	stPoint* goal_stPointp;	// Ŀ��״̬��
	stPoint* init_stPointp;	// ��ʼ״̬��

	stNode* current_stNode;//����ҵ�Ŀ��ڵ�������Ϊ��ǰ�ڵ�
	CDistance* distance_CDistance;//�����
	std::unordered_map<int,stAgentPosition>* cat;//��ײ����� Collision avoidance table

	std::priority_queue<stNode*> open_priority_queue;	//���ȶ��� �󶥶ѽṹ
	std::vector<stNode*> closed_vector;	// Closed list

	stNode* generate(stNode* p, int dir);	//����һ���ڵ���ӽڵ�
	bool is_goal_stNode(stNode* node);
	std::vector<int>* backtrace(stNode* n);	//����Ӹ��ڵ㵽n�ڵ��·��

	int* reconstruct_path(int agent, const std::vector<int>& tr);

public:
	int	expand();//ѡ��һ��f(n)��С�ĵ�	1Ŀ��ڵ� 2����ڵ�������
	int	num_expansions();//������չ�ڵ���
	int cost();	// Solution cost

	std::vector<int>* path(bool print);	//��Ŀ���м����ҵ���·��

	CSearch(int n, stPoint* init, stPoint* goal, CGridMap* g, CDistance* d = NULL,
		std::unordered_map<int,stAgentPosition>* cat = NULL);
	~CSearch();
};

inline
int CSearch::num_expansions(void) { 
	return expansions_node_number_int; 
}

inline int CSearch::cost()
{
	return cost_path_int;
}
