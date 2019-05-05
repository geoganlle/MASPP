/*
描述搜索的状态
对A*搜索的操作进行分解
分解为：
移动前pre_move[n]和移动后post_move[n]
**/

#pragma once
#include "CGridMap.h"
#include "CDistance.h"
/*
描述单个智能体的移动方向
**/
struct stMove
{
	eDirection dir_eDir;//移动的方向
	int agent_id_int;//移动的智能体编号
	stMove(eDirection dir, int p) :dir_eDir(dir), agent_id_int(p) {};
};

struct stAgentPosition {//智能体Agent的位置
	stPoint pos_stPoint;//智能体的位置
	int timestep_int;//时间步长
	int agentid_int;//智能体编号
	stAgentPosition(stPoint pos, int time, int agentid) :pos_stPoint(pos), timestep_int(time), agentid_int(agentid) {};
	stAgentPosition():pos_stPoint(0,0),timestep_int(0),agentid_int(0){};
	bool operator==(const stAgentPosition& agentpos_input) {
		return (pointEquals(&(this->pos_stPoint), agentpos_input.pos_stPoint.x, agentpos_input.pos_stPoint.y) && (agentid_int == agentpos_input.agentid_int));
	};
};
class CState
{
	const CState* parentState;//上一状态
	stPoint* pre_move_stPoint;	//移动前的位置
	stPoint* post_move_stPoint;	//移动后的位置
	short n_int; //智能体的数量
	short cost_int;//该状态的总成本

	void increment_step();//用post_move_stPoint初始化pre_move_stPoint
	stPoint* collision(stPoint* p, int agent, bool post);//碰撞检测，若有碰撞返回碰撞点，若无碰撞返回NULL(0/false)
public:
	CState(stPoint* init,int n);
	CState(int n, const CState& parent, const stMove& move);
	~CState();

	void print_to_console();

	//A*算法公式：f(n)=g(n)+h(n)
	
	int g(void);//从初始节点到该状态的代价

	int h(stPoint* goal);//曼哈顿距离启发式  Manhattan Distance Heuristic
	int h(stPoint* goal, CGridMap* g);	//真实距离启发式 True Distance Heuristic
	int h(stPoint* goal, CDistance* dist);

	int  timestep();//当前时序

	stAgentPosition movecheck(const stMove& move);//返回智能体将要移动后的位置映射

	bool* valid_moves(int, CGridMap*);//智能体的有效移动列表
	stPoint* get_stAgentPosition(int id);
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
stPoint* CState::get_stAgentPosition(int agentid)
{
	return (agentid >= 0 && agentid < n_int) ? &pre_move_stPoint[agentid] : NULL;
}