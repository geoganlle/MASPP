#include "CSearch.h"


/*����һ���ڵ���ӽڵ�*/
stNode_Search* CSearch::generate_childnode(stNode_Search* node_input, int dir_input)
{
	if (node_input==nullptr)return nullptr;

	stNode_Search* child = new stNode_Search;
	stMove move((eDirection)dir_input, node_input->agentid_short);
	expansions_node_number_int++;//�ڵ���չ��+1

	/*�������ڵ���0��������Ľڵ㣬���ڵ�ָ���Լ�*/
	child->parent_stNodep = 
		(node_input->agentid_short) ? node_input->parent_stNodep : node_input;
	child->agentid_short = 
		(node_input->agentid_short + 1 == n_agentnumber_int) ? 0 : node_input->agentid_short + 1;
	
	child->state_CStatep = 
		new CState(n_agentnumber_int, *(node_input->state_CStatep), move);

	int g = node_input->state_CStatep->g();
	int h = child->state_CStatep->h(goal_stPointp, distance_CDistance);
	child->f = (g + h < h) ? h : g + h;	//��������ֵ���intֵ
	child->dir_from_parent = dir_input;
	return child;
}

bool CSearch::is_goal_stNode(stNode_Search* node_input)
{
	if (node_input->agentid_short) return false;//Ŀ��ڵ��Ȼ����������Ϊ0�Ľڵ�

	for (int i = 0; i < n_agentnumber_int; i++) {
		stPoint* p = node_input->state_CStatep->get_pre_move_stPoint(i);
		if (goal_stPointp[i].x != p->x || goal_stPointp[i].y != p->y)
			return false;
	}
	current_stNode = node_input;//����ǰ�ڵ�����ΪĿ��ڵ�
	cost_path_int = node_input->state_CStatep->g() / n_agentnumber_int;
	std::cout << "found goal node with cost " << node_input->state_CStatep->g() / n_agentnumber_int << "!\n";
	std::cout << "\tNum Expansions = " << num_expansions() << " nodes\n";
	return true;
}

std::vector<int>* CSearch::getpath(stNode_Search* node_input)
{
	/* ����ÿ����������ƶ�����
	 *	moves[i] ��ʾ������id���ƶ��켣
	 */
	if (!node_input) return NULL;

	std::vector <int>* moves = new std::vector<int>[n_agentnumber_int];
	/*
	moves ���ʼ���ά����
	moves[0]	vector<int> //0���������·��
	moves[1]	vector<int>
	moves[2]	vector<int>
	moves[3]	vector<int>
	����
	**/
	do {
		if (!node_input->parent_stNodep)break;//������ڵ㲻������ֱ�ӷ���
		CState* parent_state = node_input->parent_stNodep->state_CStatep;

		CState* current_state = node_input->state_CStatep;
		for (int i = 0; i < n_agentnumber_int; i++) {
			stPoint* p = parent_state->get_pre_move_stPoint(i);
			stPoint* c = current_state->get_pre_move_stPoint(i);
			moves[i].push_back(dir_get(p, c));
		}
		node_input = node_input->parent_stNodep;
	} while (node_input->parent_stNodep);

	for (int i = 0; i < n_agentnumber_int; i++) {
		std::cout << "Moves for Agent " << i << std::endl;
		reverse(moves[i].begin(), moves[i].end());
		for (auto it = moves[i].begin(); it != moves[i].end(); it++) {
			std::cout << dir_to_string(*it);
			if (it != moves[i].end())std::cout << "->";
			else std::cout << std::endl;
		}
	}
	return moves;
}

int* CSearch::get_hashPoint_of_path(int agent, const std::vector<int>& path_vector)
{
	if (agent < 0 || agent >= n_agentnumber_int) return NULL;

	int* path = new int[path_vector.size() + 1];
	stPoint init_stPoint = init_stPointp[agent];

	for (int i = 0; i < path_vector.size(); i++) {
		path[i] = gridmap->hashpt(&init_stPoint);
		init_stPoint = dir_move_stPoint(&init_stPoint, path_vector[i]);
	}
	path[path_vector.size()] = gridmap->hashpt(&init_stPoint);
	return path;
}

int CSearch::expand()
{
	/* ѡ��open������f�ɱ���С�Ľڵ�*/

	stNode_Search* mynode = NULL;//����չ�ĵĽڵ�

	if (open_priority_queue.empty()) {
		std::cout << "ERROR: NULL chosen for expansion .open_priority_queue.empty() \n";
		return false;
	}

	mynode = open_priority_queue.top();
	open_priority_queue.pop();

	int my_node_f = mynode->f;
	/* �����Ŀ��ڵ�ֱ�ӷ���1 */
	if (is_goal_stNode(mynode))
		return 1;
	/* ��չ�ڵ�����򷵻�2 */
	if (num_expansions() > EXPLIM) {
		std::cout << "Exceeded expansion threshold";
		return 2;
	}
	/* ��ȡnode�ڵ���������ƶ���λ�� */
	int agentid = mynode->agentid_short;

	bool* valid_m = mynode->state_CStatep->valid_moves(agentid,gridmap);

	/* ���������ƶ� */
	int lastmove = ( mynode->parent_stNodep ) ? 
		dir_get(mynode->state_CStatep->get_pre_move_stPoint(agentid),
			mynode->parent_stNodep->state_CStatep->get_pre_move_stPoint(agentid)) :
		WAIT;

	for (int i = 0; i < DIM + 1; i++) {
		if (valid_m[i]) { //&& i != lastmove) {

			if (cat != NULL) {//��ײ���
				stAgentPosition tmp_pos = mynode->state_CStatep->get_move_AgentPosition(stMove((eDirection)i, agentid));
				if ((cat->find(tmp_pos.timestep_int)) != cat->end()) {
					auto it = cat->find(tmp_pos.timestep_int);
					if (it->second == tmp_pos) {//�ҵ���ײ
						continue;
					}
				}
			}
			open_priority_queue.push(generate_childnode(mynode,i));//����һ���ӽڵ�
		}
	}

	if (agentid) {
		delete mynode->state_CStatep;
		delete mynode;
	}
	else closed_vector.push_back(mynode);
	delete[] valid_m;
	return 0;
}

 std::vector<int>* CSearch::path(bool print=false)
{
	 if (!current_stNode) return NULL;//��ǰ�ڵ㲻���ڵĻ����ؿ�

	 std::vector<int>* pos = new std::vector<int>[n_agentnumber_int];
	 std::vector<int>* moves = getpath(current_stNode);

	 for (int i = 0; i < n_agentnumber_int; i++) {
		 if (print) std::cout << "Player " << i << std::endl;
		 int* arr = get_hashPoint_of_path(i, moves[i]);
		 for (int j = 0; j < moves[i].size() + 1; j++) {
			 if (print) std::cout << arr[j] << " ";
			 pos[i].push_back(arr[j]);
		 }
		 if (print) std::cout << std::endl;
		 delete[] arr;
	 }
	 delete[] moves;
	 return pos;	//���ص�ǰcurrent_stNode��·��
}

 CSearch::CSearch(int agentnumber,
	 stPoint* init, stPoint* goal,
	 CGridMap* g, CDistance* d,
	 std::unordered_map<int, stAgentPosition>* cat) :
	 n_agentnumber_int(agentnumber),
	 init_stPointp(init),
	 gridmap(g),
	 distance_CDistance(d),
	 cat(cat),
	 expansions_node_number_int(1),
	 current_stNode(nullptr),
	 cost_path_int(0)
{
	goal_stPointp = new stPoint[agentnumber];
	for (int i = 0; i < agentnumber; i++) {
		goal_stPointp[i] = goal[i];
	}
	stNode_Search * tmp = new stNode_Search;
	tmp->parent_stNodep = NULL;  
	tmp->state_CStatep = new CState(init, agentnumber);
	tmp->f = tmp->state_CStatep->h(goal);
	tmp->agentid_short = 0;
	open_priority_queue.push(tmp);
}

CSearch::~CSearch()
{

	delete[] goal_stPointp;

	while (!open_priority_queue.empty()) {
		stNode_Search* t = open_priority_queue.top();
		delete t->state_CStatep;
		open_priority_queue.pop();
	}
	for (auto it = closed_vector.begin(); it != closed_vector.end(); it++) {
		delete (*it)->state_CStatep;
		delete (*it);
	} 
}
