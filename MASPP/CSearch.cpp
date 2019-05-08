#include "CSearch.h"


/*生成一个节点的子节点*/
stNode* CSearch::generate(stNode* node_input, int dir_input)
{
	if (node_input == NULL|| node_input==nullptr)return nullptr;
	stNode* child = new stNode;
	stMove move((eDirection)dir_input, node_input->agentid_short);

	expansions_node_number_int++;
	/*如果输入节点是0号智能体的节点，父节点指向自己*/
	child->parent_stNodep = 
		(node_input->agentid_short) ? node_input->parent_stNodep : node_input;
	child->agentid_short = 
		(node_input->agentid_short + 1 == n_agentnumber_int) ? 0 : node_input->agentid_short + 1;
	
	child->state_CStatep = 
		new CState(n_agentnumber_int, *(node_input->state_CStatep), move);

	int g = node_input->state_CStatep->g();
	int h = child->state_CStatep->h(goal_stPointp, distance_CDistance);
	//child->f = node_input->s->g() + child->s->h(goal_stPointp, distance_CDistance);//真实距离
	child->f = (g + h < h) ? h : g + h;	//如果溢出则赋值最大int值
	child->dir_from_parent = dir_input;
	return child;
}

bool CSearch::is_goal_stNode(stNode* node_input)
{
	if (node_input->agentid_short) return false;

	for (int i = 0; i < n_agentnumber_int; i++) {
		stPoint* p = node_input->state_CStatep->get_stAgentPosition(i);
		if (goal_stPointp[i].x != p->x || goal_stPointp[i].y != p->y)
			return false;
	}
	current_stNode = node_input;	// Set goal node
	cost_path_int = node_input->state_CStatep->g() / n_agentnumber_int;
	std::cout << "Found goal with cost " << node_input->state_CStatep->g() / n_agentnumber_int << "!\n";
	std::cout << "\tNum Expansions = " << num_expansions() << " nodes\n";
	return true;
}

std::vector<int>* CSearch::backtrace(stNode* node_input)
{
	/* 返回每个智能体的移动数组
	 *	moves[i] 表示智能体id的移动轨迹
	 */
	if (!node_input) return NULL;

	std::vector <int>* moves = new std::vector<int>[n_agentnumber_int];
	do {
		CState* parent_state = node_input->parent_stNodep->state_CStatep;
		CState* current_state = node_input->state_CStatep;
		for (int i = 0; i < n_agentnumber_int; i++) {
			stPoint* p = parent_state->get_stAgentPosition(i);
			stPoint* c = current_state->get_stAgentPosition(i);
			moves[i].push_back(get_direction(p, c));
		}
		node_input = node_input->parent_stNodep;
	} while (node_input->parent_stNodep);

	for (int i = 0; i < n_agentnumber_int; i++) {
		std::cout << "Moves for Agent " << i << std::endl;
		reverse(moves[i].begin(), moves[i].end());
		for (auto it = moves[i].begin(); it != moves[i].end(); it++) {
			std::cout << dir_to_string(*it);
			if (it != moves[i].end())std::cout << "->";
			else std::cout << std::endl;;
		}
	}
	return moves;
}

int* CSearch::reconstruct_path(int agent, const std::vector<int>& tr)
{
	if (agent >= n_agentnumber_int) return NULL;

	int* path = new int[tr.size() + 1];
	stPoint init_s = init_stPointp[agent];

	for (int i = 0; i < tr.size(); i++) {
		path[i] = gridmap->hashpt(&init_s);
		init_s = getPoint_move_dir(&init_s, tr[i]);
	}
	path[tr.size()] = gridmap->hashpt(&init_s);
	return path;
}

int CSearch::expand()
{
	/* 选择open集合中f成本最小的节点*/
	stNode* node = NULL;//待拓展的的节点

	if (open_priority_queue.empty()) {
		std::cout << "ERROR: NULL chosen for expansion .open_priority_queue.empty() " << std::endl;
		return false;
	}

	node = open_priority_queue.top();
	open_priority_queue.pop();

	int myf = node->f;
	int mylen = open_priority_queue.size();
	/* 如果是目标节点直接返回1 */
	if (is_goal_stNode(node))
		return 1;
	/* 拓展节点过多则返回2 */
	if (num_expansions() > EXPLIM) {
		std::cout << "Exceeded expansion threshold";
		return 2;
	}
	/* 获取node节点智能体可移动的位置 */
	int agentid = node->agentid_short;

	bool* valid_m = node->state_CStatep->valid_moves(agentid,gridmap);

	/* 不允许反向移动 */
	int lastmove = ( node->parent_stNodep ) ? 
		get_direction(node->state_CStatep->get_stAgentPosition(agentid),
			node->parent_stNodep->state_CStatep->get_stAgentPosition(agentid)) :
		WAIT;

	for (int i = 0; i < DIM + 1; i++) {
		if (valid_m[i]) { //&& i != lastmove) {

			if (cat != NULL) {//碰撞检测
				stAgentPosition tmp_m = node->state_CStatep->movecheck(stMove((eDirection)i, agentid));
				if ((cat->find(tmp_m.timestep_int)) != cat->end()) {
					auto it = cat->find(tmp_m.timestep_int);
					if (it->second == tmp_m) {
						continue;
					}
				}
			}
			open_priority_queue.push(generate(node,i));
		}
	}

	if (agentid) {
		delete node->state_CStatep;
		delete node;
	}
	else closed_vector.push_back(node);
	delete[] valid_m;
	mylen = open_priority_queue.size();
	return 0;
}

 std::vector<int>* CSearch::path(bool print=false)
{
	 if (!current_stNode) return NULL;

	 std::vector<int>* pos = new std::vector<int>[n_agentnumber_int];
	 std::vector<int>* moves = backtrace(current_stNode);

	 for (int i = 0; i < n_agentnumber_int; i++) {
		 if (print) std::cout << "Player " << i << std::endl;
		 int* arr = reconstruct_path(i, moves[i]);
		 for (int j = 0; j < moves[i].size() + 1; j++) {
			 if (print) std::cout << arr[j] << " ";
			 pos[i].push_back(arr[j]);
		 }
		 if (print) std::cout << std::endl;
		 delete[] arr;
	 }
	 delete[] moves;
	 return pos;	//返回一个智能体搜索的所有路径
}

CSearch::CSearch(int agentnumber, 
	stPoint* init, stPoint* goal,
	CGridMap* g, CDistance* d,
	std::unordered_map<int, stAgentPosition>* cat):
	n_agentnumber_int(agentnumber), init_stPointp(init),
	gridmap(g), distance_CDistance(d), cat(cat), expansions_node_number_int(1) {

	current_stNode = NULL;

	goal_stPointp = new stPoint[agentnumber];
	for (int i = 0; i < agentnumber; i++) {
		goal_stPointp[i] = goal[i];
	}
	stNode * tmp = new stNode;
	tmp->parent_stNodep = NULL;  
	tmp->state_CStatep = new CState(init, agentnumber);
	tmp->f = tmp->state_CStatep->h(goal);
	tmp->agentid_short = 0;
	open_priority_queue.push(tmp);
}

CSearch::~CSearch()
{
	/*
	仅删除在这个类中创建(new)的对象
	**/
	delete[] goal_stPointp;
	/*
	init_stPointp\distance_CDistance删除可能会导致未知问题
	delete[] init_stPointp;
	delete distance_CDistance;
	**/
	while (!open_priority_queue.empty()) {
		stNode* t = open_priority_queue.top();
		delete t->state_CStatep;
		open_priority_queue.pop();
	}
	for (auto it = closed_vector.begin(); it != closed_vector.end(); it++) {
		delete (*it)->state_CStatep;
		delete (*it);
	} 
}
