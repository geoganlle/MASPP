/*
program file No.4
����ϵͳ������״̬
���A*�㷨��������ֽ⣨Operator Decomposition��OD��
�ֽ��ÿ��Agentһ��״̬��State��

˵��
ÿ��������ƻ��ƶ���λ��pre_move[n]
ÿ���������ƶ����λ��post_move[n]

Written by Geoganlle Goo
**/

#pragma once
#include "CGridMap.h"
#include "CDistance.h"

/*
����һ���������һ���ƶ���Ϊ
**/
struct stMove
{
	eDirection dir_eDir;//�ƶ��ķ���
	int agent_id_int;//�ƶ�����������
	stMove(eDirection dir, int p) :dir_eDir(dir), agent_id_int(p) {};
};

/*
���������������λ�ã�ʱ��
**/
struct stAgentPosition {//������Agent��λ��
	stPoint pos_stPoint;//�������λ��
	int timestep_int;//ʱ�䲽��*��ʱ
	int agentid_int;//��������
	stAgentPosition(stPoint pos, int time, int agentid) :pos_stPoint(pos), timestep_int(time), agentid_int(agentid) {};
	stAgentPosition():pos_stPoint(0,0),timestep_int(0),agentid_int(0){};
	bool operator==(const stAgentPosition& agentpos_input) {
		return this->pos_stPoint==agentpos_input.pos_stPoint && this->agentid_int == agentpos_input.agentid_int;
	};
};

class CState
{
	const CState* parentState;//��һ״̬
	stPoint* pre_move_stPoint;	//�ƻ��ƶ���λ��
	stPoint* post_move_stPoint;	//�ƶ����λ��
	short n_agent_number_int; //����������� SHRT_MAX = 32767
	int cost_int;//��״̬���ܳɱ�
	
	void increment_step();//�ƶ�һ�� pre_move_stPoint=post_move_stPoint
	
	/*
	��ײ��⺯��
	����ײ�Ļ�������ײ��
	����ײ����NULL
	 post :��post_move�Ƚ�(true)������pre_move�Ƚ�(false)
	**/
	stPoint* collision_detection(stPoint* p, int agent, bool post);
public:

	CState(stPoint* init,int n_agent_number);
	CState(int n_agent_number, const CState& parent, const stMove& move);
	~CState();

	void print_to_console();

	//A*�㷨��ʽ��f(n)=g(n)+h(n)
	
	int g(void);//�ӳ�ʼ�ڵ㵽��״̬�Ĵ���

	int h(stPoint* goal);//�����پ�������ʽ  Manhattan Distance Heuristic
	int h(stPoint* goal, CGridMap* g);	//��ʵ��������ʽ True Distance Heuristic
	int h(stPoint* goal, CDistance* dist);

	int  timestep();//ƽ��ÿ��������Ŀ���(state ״̬����ĳ���)

	stAgentPosition get_move_AgentPosition(const stMove& move);//����״̬��post_moveһ���ƶ�֮���λ��

	bool* valid_moves(int, CGridMap*);//����pre_post����Ч�ƶ����� (9��ѡ��,8������+wait)
	
	stPoint* get_pre_move_stPoint(int id);
};
inline
void CState::increment_step() {
	for (int i = 0; i < n_agent_number_int; i++) {
		pre_move_stPoint[i] = post_move_stPoint[i];
	}
}

inline
int CState::g() {
	return cost_int;
}
//����ֵ�������پ���
inline int CState::h(stPoint* goal)
{
	int cost = 0;
	for (int i = 0; i < n_agent_number_int; i++) {
		int xd = goal[i].x - post_move_stPoint[i].x;
		int yd = goal[i].y - post_move_stPoint[i].y;
		cost += (xd > 0) ? xd : -xd;
		cost += (yd > 0) ? yd : -yd;
	}
	return cost;
}

inline
int CState::timestep()
{
	return cost_int/n_agent_number_int;
}

inline
stPoint* CState::get_pre_move_stPoint(int agentid)
{
	return (agentid >= 0 && agentid < n_agent_number_int) ? &pre_move_stPoint[agentid] : NULL;
}