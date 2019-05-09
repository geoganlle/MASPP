#include "CState.h"

stPoint* CState::collision(stPoint* p, int agent, bool post)
{
	/* 如果没有碰撞的话返回空值. 有碰撞的话返回一个指向碰撞智能体的指针
	 post :与下一次移动比较(true)还是与上一次移动比较(false)
	**/
	stPoint* cmp_stPoint = (post) ? post_move_stPoint : pre_move_stPoint;
	for (int i = 0; i < n_int; i++) {
		if (i != agent && p->x == cmp_stPoint[i].x && p->y == cmp_stPoint[i].y)
			return &cmp_stPoint[i];
		if (post && i == agent)	//检测agent之前的agent会不会和p点碰撞
			return NULL;
	}
	return NULL; 
}

CState::CState(stPoint* init, int n)
{
	parentState = nullptr;
	n_int = n;
	cost_int = 0;
	pre_move_stPoint = new stPoint[n];
	post_move_stPoint = new stPoint[n];
	for (int i = 0; i < n; i++) {
		pre_move_stPoint[i] = init[i];
		post_move_stPoint[i] = init[i];
	}
}

CState::CState(int n, const CState& parent, const stMove& move):n_int(n)
{
	int agent_id = move.agent_id_int;
	cost_int = parent.cost_int + 1;
	if (agent_id) {//如果不是第一个智能体
		this->parentState = parent.parentState;
	}
	else {
		this->parentState =&parent;
	}
	pre_move_stPoint = new stPoint[n];
	post_move_stPoint = new stPoint[n];
	for (int i = 0; i < n; i++) {
		pre_move_stPoint[i] = parent.pre_move_stPoint[i];
		post_move_stPoint[i] = parent.post_move_stPoint[i];
	}
	post_move_stPoint[agent_id] = dir_move_stPoint(&post_move_stPoint[agent_id],move.dir_eDir);
	if (agent_id == n - 1) {//如果是最后一个智能体，则所有启动前位置等于移动后位置
		increment_step();
	}
}

CState::~CState()
{
	delete[] pre_move_stPoint;
	delete[] post_move_stPoint;
}

void CState::print_to_console()
{
	for (int i = 0; i < n_int; i++) {
		std::cout << "Pre =(" << pre_move_stPoint[i].y << "," << pre_move_stPoint[i].x << ") ";
		std::cout << "Post=(" << post_move_stPoint[i].y << "," << post_move_stPoint[i].x << ")";
		std::cout << std::endl;
	}
}

//启发式函数：真实距离 所有智能体下一状态到目标点的距离和
int CState::h(stPoint * goal, CGridMap * gridmap)
{
	int cost_temp = 0;
	for (int i = 0; i < n_int; i++) {
		stPoint p_init = post_move_stPoint[i];
		stPoint p_goal = goal[i];
		CBfs bfs(&p_init, &p_goal, gridmap);
		if (bfs.get_soln_cost_int() == INT_MAX)
			return INT_MAX;
		cost_temp += bfs.get_soln_cost_int();
	}
	return cost_temp;
}

int CState::h(stPoint* goal, CDistance* dist)
{
	int cost_temp = 0;
	for (int i = 0; i < n_int; i++) {
		stPoint p_init = post_move_stPoint[i];
		stPoint p_goal = goal[i];
		int temp = dist->getDistance(p_init, p_goal);
		if (temp == INT_MAX)
			return INT_MAX;
		cost_temp += temp;
	}
	return cost_temp;
}

stAgentPosition CState::get_move_AgentPosition(const stMove& move)
{
	int agentid = move.agent_id_int;
	int dir = move.dir_eDir;
	stPoint after_move_point = dir_move_stPoint(&post_move_stPoint[agentid], dir);
	int time = (agentid == n_int - 1) ? timestep() + 1 : timestep();
	return stAgentPosition(after_move_point, time, agentid);
}

bool* CState::valid_moves(int agentid, CGridMap* gridmap)
{
	stPoint* pt = get_pre_move_stPoint(agentid);
	if (!pt) return NULL;

	bool* vld = new bool[DIM + 1];

	vld[WAIT] = (collision(pt, agentid, true)) ? false : true;//智能体不移动是否有碰撞

	bool* neighhors = gridmap->getNeighbor(*pt);
	if (!neighhors)
		return vld;

	for (int i = 0; i < DIM; i++) {
		if (neighhors[i]) {
			stPoint move_Point = dir_move_stPoint(pt, i);
			if (collision(&move_Point, agentid, true)) {
				vld[i] = false;
			}
			else { 
				vld[i] = true; 
			}
		}
		else vld[i] = false;
	}
	delete[] neighhors;
	return vld;
}
