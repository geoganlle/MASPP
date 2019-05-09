/*
program file No.4
描述搜索的状态
智能体上一步的位置pre_move[n]
智能体下一步的位置post_move[n]
**/

#pragma once
#include "CGridMap.h"
#include "CDistance.h"

/*
描述系统搜索的状态
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
	int timestep_int;//时间步长
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
	stPoint* pre_move_stPoint;	//移动前的位置
	stPoint* post_move_stPoint;	//移动后的位置
	short n_int; //智能体的数量 SHRT_MAX = 32767
	int cost_int;//该状态的总成本
	
	void increment_step();//移动一步 用post_move_stPoint复制pre_move_stPoint
	
	/*
	碰撞检测，若有碰撞返回碰撞点，若无碰撞返回NULL(0/false) 
	post :与下一次移动比较(true)还是与上一次移动比较(false)
	**/
	stPoint* collision(stPoint* p, int agent, bool post);
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

	int  timestep();//平均每个智能体的花销

	stAgentPosition get_move_AgentPosition(const stMove& move);//返回智能体将要移动后的位置映射

	bool* valid_moves(int, CGridMap*);//智能体的有效移动列表 9种选择
	
	stPoint* get_pre_move_stPoint(int id);
};
inline
void CState::increment_step() {
	for (int i = 0; i < n_int; i++) {
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
	for (int i = 0; i < n_int; i++) {
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
	return cost_int/n_int;
}

inline
stPoint* CState::get_pre_move_stPoint(int agentid)
{
	return (agentid >= 0 && agentid < n_int) ? &pre_move_stPoint[agentid] : NULL;
}