#include "CAgent.h"



CAgent::CAgent()
{
}


CAgent::~CAgent()
{
}

Mapf::Mapf(int n, stPoint* s_init, stPoint* s_goal, CGridMap* gd) : n_agent_number_int(n), grid(gd) {
	e_expansions_int = 0;
	c_collisions_int = 0;
	max_cost_int = 0;
	time(&start_t);
	dlt = new CDistance(grid);
	cat = new std::unordered_map<int, stAgentPosition>[n];

	/* Begin with n singleton groups */
	for (int i = 0; i < n; i++) {
		// Create an agent_t struct for each agent and group for ea agent
		agentlist.push_back(stAgent(i, s_init[i], s_goal[i]));
		std::vector<int> list;
		list.push_back(i);
		groups.push_back(list);
	}
}

Mapf::~Mapf() {
	delete dlt;
	delete[] cat;
}

// Resolve conflicts among groups. If conflicts exist, merge groups
int Mapf::resolve_conflicts(void) {
	//cout << "resolve_conflicts start\n";
	std::cout << "..........................\n";

	// For each group find independent soln and look for conflicts
	bool conflicts = false;	// Indicates whether conflicts found
	std::vector<int>** id_paths = new std::vector<int> * [groups.size()];
	int oldlen = groups.size();

	int num_groups = groups.size();

	for (int i = 0; i < num_groups; i++) {	// For ea group
		int len = groups[i].size();
		stPoint* s_init = new stPoint[len];
		stPoint* s_goal = new stPoint[len];

		// Populate init and goal states for members of the group	
		for (int j = 0; j < len; j++) {
			int agent_id = groups[i][j];
			s_init[j] = agentlist[agent_id].init;
			s_goal[j] = agentlist[agent_id].goal;
		}
		// Find solution on group
		std::cout << "starting search groupsize " << len << std::endl;
		CSearch s(len, s_init, s_goal, grid, dlt, &cat[i]);
		int result = 0;
		do {
			result = s.expand();
		} while (!result);

		max_cost_int = (s.cost() > max_cost_int) ? s.cost() : max_cost_int;
		//cout << "***********\nDone\n";

		if (result == 2) {
			delete[] s_init;
			delete[] s_goal;
			delete[] id_paths;
			return 2;
		}

		e_expansions_int += s.num_expansions();

		std::vector<int>* g_paths = s.path(false);	// Get the soln path
		id_paths[i] = g_paths;

		/* TODO: Determine conflicts across groups
			then proceed to record positions if no conflicts are found
		*/

		std::cout << "Group " << i << std::endl;
		if (g_paths) {
			for (int j = 0; j < len; j++) {	// For ea agent in the group

				int agent_id = groups[i][j];
				std::cout << "   Agent: " << agent_id << "\t";
				for (int k = 0; k < g_paths[j].size(); k++) {	// For ea move
					std::cout << g_paths[j][k] << " ";
				}
				std::cout << std::endl;
			}
		}
		//cout << "resolve_conflicts end\n";

		delete[] s_init;
		delete[] s_goal;
	}

	//cout << "group conflict resolution start\n";

	for (int i = 0; i < num_groups - 1; i++) {
		int len1 = groups[i].size();
		for (int j = i + 1; j < num_groups && !conflicts; j++) {
			// For ea pair of groups
			int len2 = groups[j].size();

			int numc = 0;
			/* If there is a path conflict between groups i and j */
			if (numc = group_conflict(id_paths[i], id_paths[j], len1, len2)) {
				conflicts = true;
				c_collisions_int++;

				std::cout << "Conflict found between Group " << i << " and " << j << std::endl;
				if (numc == 1 && cat[i].size() < 2 &&
					cat[i].find(lastConflict.timestep_int) == cat[i].end()) {
					std::cout << "Adding conflict avoidance entry\n";
					std::cout << "\tGroup: " << i << "\tAgent: " <<
						lastConflict.agentid_int << "\tTime: " << 
						lastConflict.timestep_int << std::endl;
					cat[i].insert({ {lastConflict.timestep_int, lastConflict} });
				}
				else {
					/* Merge the groups */
					std::cout << "Merging\n";
					std::vector<int>* g1 = &groups[i];
					std::vector<int>* g2 = &groups[j];
					cat[i].clear();
					cat[j].clear();
					g1->insert(g1->end(), g2->begin(), g2->end());

					auto it = groups.begin() + j;
					groups.erase(it);
				}

			}
		}
	}
	if (!conflicts) {
		time_t end_t = time(NULL);
		elapse_t = difftime(end_t, start_t);
	}

	//cout << "Group conflict resolution end\n";

	for (int i = 0; i < num_groups; i++)
		delete[] id_paths[i];

	delete[] id_paths;
	return (conflicts) ? 1 : 0;
}

int
Mapf::group_conflict(std::vector<int> * g1, std::vector<int> * g2, int len1, int len2) {
	if (!(g1 && g2)) return false;

	/* Use len of shortest path */
	int plen = (g1[0].size() > g2[0].size()) ? g2[0].size() : g1[0].size();
	int numc = 0;

	/* Take cross product
	 * Consider all pairs of agents (u, v) where u in g1 and v in g2 */
	for (int i = 0; i < len1; i++) {
		for (int j = 0; j < len2; j++) {
			int pc = path_conflict(&g1[i], &g2[j], plen);

			if (pc) lastConflict.agentid_int = i;

			numc += pc;
			//if (path_conflict(&g1[i], &g2[j], plen))
				//return true;	
		}
	}
	/* No path conflict between groups g1 and g2*/
	//return false;
	if (!numc) return 0;	// No conflicts
	return (numc > 1) ? 2 : 1;	// 2 = More than 1 conflict, 1 = just one
}

int
Mapf::path_conflict(std::vector<int> * p1, std::vector<int> * p2, int len) {
	int numc = 0;
	/* Return # of conflicts on path of 2 agents */
	for (int i = 0; i < len; i++)
		if (p1->at(i) == p2->at(i)) {
			std::cout << "Conflict at t = " << i << ". Value is " << p1->at(i)
				<< std::endl;
			numc++;
			lastConflict.pos_stPoint = grid->unhash(p1->at(i));
			lastConflict.timestep_int = i;
		}

	return numc;
}
