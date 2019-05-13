/*
program file No.4
描述系统搜索的状态
针对A*算法的运算符分解（Operator Decomposition，OD）
分解成每个Agent一个状态（State）

说明
每个智能体计划移动的位置pre_move[n]
每个智能体移动后的位置post_move[n]

Written by Geoganlle Goo
**/

#pragma once
#include "CGridMap.h"
#include "CDistance.h"

/*
描述一个智能体的一次移动行为
**/
struct stMove
{
	eDirection dir_eDir;//移动的方向
	int agent_id_int;//移动的智能体编号
	stMove(eDirection dir, int p) :dir_eDir(dir), agent_id_int(p) {};
};

/*
描述单个智能体的位置，时长
**/
struct stAgentPosition {//智能体Agent的位置
	stPoint pos_stPoint;//智能体的位置
	int timestep_int;//时间步长*耗时
	int agentid_int;//智能体编号
	stAgentPosition(stPoint pos, int time, int agentid) :pos_stPoint(pos), timestep_int(time), agentid_int(agentid) {};
	stAgentPosition():pos_stPoint(0,0),timestep_int(0),agentid_int(0){};
	bool operator==(const stAgentPosition& agentpos_input) {
		return this->pos_stPoint==agentpos_input.pos_stPoint && this->agentid_int == agentpos_input.agentid_int;
	};
};

class CState
{
	const CState* parentState;//上一状态
	stPoint* pre_move_stPoint;	//计划移动的位置
	stPoint* post_move_stPoint;	//移动后的位置
	short n_agent_number_int; //智能体的数量 SHRT_MAX = 32767
	int cost_int;//该状态的总成本
	
	void increment_step();//移动一步 pre_move_stPoint=post_move_stPoint
	
	/*
	碰撞检测函数
	有碰撞的话返回碰撞点
	无碰撞返回NULL
	 post :与post_move比较(true)还是与pre_move比较(false)
	**/
	stPoint* collision_detection(stPoint* p, int agent, bool post);
public:

	CState(stPoint* init,int n_agent_number);
	CState(int n_agent_number, const CState& parent, const stMove& move);
	~CState();

	void print_to_console();

	//A*算法公式：f(n)=g(n)+h(n)
	
	int g(void);//从初始节点到该状态的代价

	int h(stPoint* goal);//曼哈顿距离启发式  Manhattan Distance Heuristic
	int h(stPoint* goal, CGridMap* g);	//真实距离启发式 True Distance Heuristic
	int h(stPoint* goal, CDistance* dist);

	int  timestep();//平均每个智能体的开销(state 状态链表的长度)

	stAgentPosition get_move_AgentPosition(const stMove& move);//返回状态中post_move一次移动之后的位置

	bool* valid_moves(int, CGridMap*);//返回pre_post的有效移动方向 (9种选择,8个方向+wait)
	
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
//启发值：曼哈顿距离
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