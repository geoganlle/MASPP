#include "CMultiAgentSystem.h"

CMultiAgentSystem::CMultiAgentSystem(int n, stPoint* s_init, stPoint* s_goal, CGridMap* gd) : n_agent_number_int(n), gridmap_CGridMap(gd) {
	e_expansions_int = 0;
	c_collisions_int = 0;
	elapse_t = 0;
	max_cost_int = 0;
	//time(&start_t);
	start_t = clock();
	distance_CDistance = new CDistance(gridmap_CGridMap);
	cat = new std::unordered_map<int, stAgentPosition>[n];

	for (int i = 0; i < n; i++) {
		//��ÿ�������崴���ṹ��ͽ��������·������
		agentlist.push_back(stAgent(i, s_init[i], s_goal[i]));
		std::vector<int> list;
		list.push_back(i);
		groups.push_back(list);
	}
}

CMultiAgentSystem::~CMultiAgentSystem() {
	delete distance_CDistance;
	delete[] cat;
}

//���С��֮��ĳ�ͻ��������ڳ�ͻ����ϲ��� δ����򷵻�2
int CMultiAgentSystem::resolve_conflicts(void) {

	//Ϊÿ��С���ҵ������Ľ��������Ѱ�ҳ�ͻ
	bool conflicts = false;	// ָʾ�Ƿ��ҵ���ͻ

	int num_groups = groups.size();
	std::vector<int>** id_paths = new std::vector<int> * [num_groups];

	for (int i = 0; i < num_groups; i++) {
		int len = groups[i].size();
		stPoint* s_init = new stPoint[len];
		stPoint* s_goal = new stPoint[len];
		/*
		groups 
		������  1	
				2
				3
				4
		
		*/
		//������Ա�ĳ�ʼ״̬��Ŀ��״̬
		for (int j = 0; j < len; j++) {
			int agent_id = groups[i][j];
			s_init[j] = agentlist[agent_id].init;
			s_goal[j] = agentlist[agent_id].goal;
		}

		std::cout << "starting search group: "<<i<<" size: " << len << std::endl;
		CSearch s(len, s_init, s_goal, gridmap_CGridMap, distance_CDistance, &cat[i]);
		int result = 0;
		do {
			result = s.expand();
		} while (!result);

		max_cost_int = (s.get_cost_int() > max_cost_int) ? s.get_cost_int() : max_cost_int;

		if (result == 2) {//�ڵ����
			delete[] s_init;
			delete[] s_goal;
			delete[] id_paths;
			return 2;
		}

		e_expansions_int += s.get_num_expansions_int();

		std::vector<int>* g_paths = s.path(false);	// Get the soln path

		id_paths[i] = g_paths;

		/* 
		ȷ����֮��ĳ�ͻ�����δ���ֳ�ͻ���������¼λ�á�
		*/

		std::cout << "Group " << i << std::endl;
		if (g_paths) {
			for (int j = 0; j < len; j++) {	// For ea agent in the group
				int agent_id = groups[i][j];
				std::cout << "\tAgent: " << agent_id << "\t";
				for (int k = 0; k < g_paths[j].size(); k++) {	// For ea move
					std::cout << g_paths[j][k] << "->";
				}
				std::cout << std::endl;
			}
		}

		delete[] s_init;
		delete[] s_goal;
	}

	//cout << "group conflict resolution start\n";

	for (int i = 0; i < num_groups - 1; i++) {
		int len1 = groups[i].size();
		for (int j = i + 1; j < num_groups && !conflicts; j++) {
			int len2 = groups[j].size();

			int numc = 0;
			/*���I���J��֮�����·����ͻ*/
			if (numc = group_conflict(id_paths[i], id_paths[j], len1, len2)) {
				conflicts = true;
				c_collisions_int++;

				std::cout << "Conflict found between Group " << i << " and " << j << std::endl;
				if (numc == 1 && cat[i].size() < 2 &&
					cat[i].find(lastPosition_AgentPosition.timestep_int) == cat[i].end()) {
					std::cout << "Adding conflict avoidance entry\n";
					std::cout << "\tGroup: " << i << "\tAgent: " <<
						lastPosition_AgentPosition.agentid_int << "\tTime: " << 
						lastPosition_AgentPosition.timestep_int << std::endl;
					cat[i].insert({ {lastPosition_AgentPosition.timestep_int, lastPosition_AgentPosition} });
				}
				else {
					/* �ϲ��� */
					std::cout << "Merging\n";
					std::vector<int>* g1 = &groups[i];
					std::vector<int>* g2 = &groups[j];
					cat[i].clear();
					cat[j].clear();
					g1->insert(g1->end(), g2->begin(), g2->end());
					//������j��
					auto it = groups.begin() + j;
					groups.erase(it);
				}

			}
		}
	}
	if (!conflicts) {
		clock_t end_t = clock();
		elapse_t = difftime(end_t, start_t);
	}

	//cout << "Group conflict resolution end\n";

	for (int i = 0; i < num_groups; i++)
		delete[] id_paths[i];

	delete[] id_paths;
	return (conflicts) ? 1 : 0;
}

int CMultiAgentSystem::group_conflict(std::vector<int> * g1, std::vector<int> * g2, int len1, int len2) {
	if (!(g1 && g2)) return false;

	/*ȡ��������ĳ��� */
	int shortlen = (g1[0].size() > g2[0].size()) ? g2[0].size() : g1[0].size();
	int numc = 0;

	for (int i = 0; i < len1; i++) {
		for (int j = 0; j < len2; j++) {
			int pc = path_conflict(&g1[i], &g2[j], shortlen);
			if (pc) lastPosition_AgentPosition.agentid_int = i;
			numc += pc;
		}
	}

	if (!numc) return 0;	// û�г�ͻ
	return (numc > 1) ? 2 : 1;	// 2�����������ϳ�ͻ 1ֻ��һ����ͻ
}

int CMultiAgentSystem::path_conflict(std::vector<int> * p1, std::vector<int> * p2, int len) {
	int numc = 0;
	for (int i = 0; i < len; i++) {
		if (p1->at(i) == p2->at(i)) {
			std::cout << "Conflict at t = " << i << ". Value is " << p1->at(i) << "\n";
			numc++;
			lastPosition_AgentPosition.pos_stPoint = gridmap_CGridMap->unhash(p1->at(i));
			lastPosition_AgentPosition.timestep_int = i;
		}
	}

	return numc;
}
