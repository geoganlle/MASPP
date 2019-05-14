#include "CState.h"

stPoint* CState::collision_detection(stPoint* p, int agent, bool post)
{
	/*
	碰撞检测函数
	有碰撞的话返回碰撞点
	无碰撞返回NULL
	 post :与post_move比较(true)还是与pre_move比较(false)
	**/
	stPoint* cmp_stPoint = (post) ? post_move_stPoint : pre_move_stPoint;
	for (int i = 0; i < n_agent_number_int; i++) {
		if (i != agent && p->x == cmp_stPoint[i].x && p->y == cmp_stPoint[i].y)
			return &cmp_stPoint[i];
		if (post && i == agent)	//检测agent之前的agent会不会和p点碰撞
			return nullptr;
	}
	return nullptr;
}

CState::CState(stPoint* init, int n):n_agent_number_int(n)
{
	parentState = nullptr;
	cost_int = 0;
	pre_move_stPoint = new stPoint[n];
	post_move_stPoint = new stPoint[n];
	for (int i = 0; i < n; i++) {
		pre_move_stPoint[i] = init[i];
		post_move_stPoint[i] = init[i];
	}
}

CState::CState(int n, const CState& parent, const stMove& move):n_agent_number_int(n)
{
	//状态间通过一个智能体的一次移动为转移
	int agent_id = move.agent_id_int;
	cost_int = parent.cost_int + 1;
	/*
	生成顺序 state(父节点)、state(agent 0)、state(agent 1)、state(agent 2)
	逻辑结构：
					/	->state(agent 0)
	state(父节点)	-	->state(agent 1)
					\	->state(agent 2)
					\	...
	即	不同agent的所有节点连在同一个父节点上
	
	*/		
	if (agent_id) {//如果不是第一个智能体
		if (!parent.parentState) {//检测父节点的父节点是否存在
			this->parentState = parent.parentState;
		}
		else {
			//std::cerr<<"Error : Can not construct CState with no parentState.  "<<std::endl;
			this->parentState = nullptr;
			//return;
		}
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
	if (agent_id == n - 1) {//如果是最后一个智能体，移动完毕
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
	for (int i = 0; i < n_agent_number_int; i++) {
		std::cout << "Pre =(" << pre_move_stPoint[i].y << "," << pre_move_stPoint[i].x << ") ";
		std::cout << "Post=(" << post_move_stPoint[i].y << "," << post_move_stPoint[i].x << ")";
		std::cout << std::endl;
	}
}

//启发式函数：真实距离 所有智能体预计移动下到目标点的距离和
int CState::h(stPoint * goal, CGridMap * gridmap)
{
	int cost_temp = 0;
	for (int i = 0; i < n_agent_number_int; i++) {
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
	for (int i = 0; i < n_agent_number_int; i++) {
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
	int time = (agentid == n_agent_number_int - 1) ? timestep() + 1 : timestep();
	//如果是最后一个智能体的移动则时长+1表示所有智能体完成了一次状态的转移
	return stAgentPosition(after_move_point, time, agentid);
}

bool* CState::valid_moves(int agentid, CGridMap* gridmap)
{
	stPoint* pt = get_pre_move_stPoint(agentid);
	if (!pt) return NULL;
	/*该智能体可移动的位置数组*/
	bool* result_boolp = new bool[DIM + 1];

	result_boolp[WAIT] = (collision_detection(pt, agentid, true)) ? false : true;//智能体不移动是否有碰撞

	bool* neighhors = gridmap->getNeighbor(*pt);
	if (!neighhors)
		return result_boolp;

	for (int i = 0; i < DIM; i++) {
		if (neighhors[i]) {
			stPoint move_Point = dir_move_stPoint(pt, i);
			if (collision_detection(&move_Point, agentid, true)) {
				result_boolp[i] = false;
			}
			else { 
				result_boolp[i] = true; 
			}
		}
		else result_boolp[i] = false;
	}
	delete[] neighhors;
	return result_boolp;
}
