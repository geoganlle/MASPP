/*
程序的入口:
主函数

梦开始的地方
**/

#include <iostream>
#include"CDistance.h"
#include"CAgent.h"
#include <string>
#include <iomanip>
/*预处理检测路径是否存在时的开销限制
后期可优化*为依据地图特征计算的开销估值
*/
#define MAX_TOUR 10000

using namespace std;

bool mapftest(string testfile);//从文件中读取数据测试
stAgentSystem run_mapf(string path_g, string path_a);//运行一个测试实例
stPoint** readpos_agent(string pathname, int& n);//从文件中读取agent信息
bool chksolution(int init[], int goal[], int len, CGridMap* g);//预检查路径是否存在

int main() {
	//CGridMap g("../map/4x4map0.txt");
	//CBfs* bfs = new CBfs(&stPoint(0,3),&stPoint(0,0),&g);
	//int a = bfs->get_soln_cost_int();
	//cout << a;
	/*
	CDistance* d = new CDistance(&g);
	d->printDistanceTable();
	d->~CDistance();
	cout << "Done" << endl;
	*/
	mapftest("../tests/t6.test" );
	cout << "Done" << endl;
	return 0;
}

bool mapftest(string testfile) {
	string line;
	ifstream file(testfile);
	int pos = 0, total = 0;	//测试数量
	double avg_exp = 0;
	int max_exp = 0;
	int min_exp = INT_MAX;
	time_t max_t = 0;
	double avg_t = 0;
	double min_t = numeric_limits<double>::infinity();

	vector<stAgentSystem> mapf_tests;

	if (!file.is_open()) return false;

	while (getline(file, line)) {
		if (line[0] == '#') continue;


		string grid_path = line;
		string agent_path;
		getline(file, agent_path);

		cerr << "Beginning instance " << total << endl;
		cout << "grid path= " + grid_path + "\t agent path= " + agent_path + "\n";

		stAgentSystem tmp = run_mapf(grid_path, agent_path);
		mapf_tests.push_back(tmp);
		if (tmp.canbesolved_bool)
			pos++;

		else cout << "Cannot solve instance " << total << endl;

		total++;
	}

	file.close();
	cout << "###############################################\n";
	cout << "Solved " << pos << "/" << total << " instances\n";

	int i = 0;
	cerr << "Instance,Dim,k,Time,Collisions,Expansions,Cost\n";
	for (auto it = mapf_tests.begin(); it != mapf_tests.end(); it++) {
		if (it->canbesolved_bool) {
			cout << "Instance " << i++ << endl;
			cout << "\tGrid x=" << it->dim.x << ", y=" << it->dim.y << endl;
			cout << fixed;
			cout << "\tNum agents = " << it->num_agents_int << endl;
			cout << "\tTime = ";
			cout << fixed<<setprecision(8) << it->time_timet << "s\n";
			cout << "\tCollisions = " << it->num_collisions_int << endl;
			cout << "\tNum expansions = " << it->num_expansions_int << endl;

			min_t = (it->time_timet < min_t) ? it->time_timet : min_t;
			max_t = (it->time_timet > max_t) ? it->time_timet : max_t;
			min_exp = (it->num_expansions_int < min_exp) ? it->num_expansions_int : min_exp;
			max_exp = (it->num_expansions_int > max_exp) ? it->num_expansions_int : max_exp;
			avg_exp += it->num_expansions_int;
			avg_t += it->time_timet;


			// Use cerr to print csv formatted data
			cerr << fixed;
			cerr << setprecision(3);
			cerr << i << "," << it->dim.x << "x" << it->dim.y << ",";
			cerr << it->num_agents_int << "," << (double)it->time_timet << ",";
			cerr << it->num_collisions_int << "," << it->num_expansions_int << ",";
			cerr << it->system_cost_int << endl;
		}
	}

	avg_exp = avg_exp / pos;
	avg_t = avg_t / pos;

	cout << "###############################################\n";
	cout << fixed;
	cout << setprecision(2);
	cout << "Average Node exp = " << avg_exp << endl;
	cout << "\tMin = " << min_exp << "\tMax = " << max_exp << endl;
	cout << fixed;
	cout << setprecision(4);
	cout << "Average solution time = " << avg_t << endl;
	cout << "\tMin = " << min_t << "\tMax = " << max_t << endl;

	return true;
}
stAgentSystem run_mapf(string path_g, string path_a) {

	int num_agents = -1;
	path_a = "../agents/" + path_a;
	stPoint** states = readpos_agent(path_a, num_agents);
	CGridMap grid("../grids/" + path_g);

	stAgentSystem info(num_agents,0, 0,0, 0, false, grid.getDim());

	if (!chksolution((int*)states[0], (int*)states[1], num_agents, &grid))
		return info;

	CAgentSystem m(num_agents, states[0], states[1], &grid);

	int res;
	while (res = m.resolve_conflicts()) {
		if (res == 2) {
			delete[] states[0];
			delete[] states[1];
			delete[] states;
			cout << "Failed to solve!\n";
			return info;
		}
	}
	cout << "\tTotal Nodes Expanded: " << m.num_expansions() << endl;
	cout << fixed;
	cout << "\tTotal Time " << setprecision(8) << m.get_time() << "s\n";

	info.canbesolved_bool = true;
	info.num_expansions_int = m.num_expansions();
	info.time_timet = m.get_time();
	info.num_collisions_int = m.get_collisions();
	info.system_cost_int = m.cost();

	delete[] states[0];
	delete[] states[1];
	delete[] states;

	return info;
}
stPoint** readpos_agent(string pathname, int& n) {
	/*	@pathname = pathname to file
		@n = set the number of agents
	*/
	ifstream file(pathname);

	if (file.is_open()) {
		int num_agents;
		stPoint** arr = new stPoint * [2];
		file >> num_agents;	// First line should contain the num of agents
		arr[0] = new stPoint[num_agents];	// Initial state
		arr[1] = new stPoint[num_agents];	// Goal states

		for (int i = 0; i < num_agents; i++) {
			file >> arr[0][i].x;
			file >> arr[0][i].y;
			file >> arr[1][i].x;
			file >> arr[1][i].y;
		}
		n = num_agents;
		return arr;
	}
	else {
		cout << "Cannot open file " + pathname + "\n";
		return NULL;
	}
}
bool chksolution(int init[], int goal[], int len, CGridMap* g) {
	for (int i = 0; i < len; i += 2) {
		CBfs bfs((stPoint*)& init[i], (stPoint*)& goal[i], g);
		if (bfs.get_soln_cost_int() > MAX_TOUR) return false;
		cout << "Found path for " << i << endl;
	}
	cout << "Checked solutions OK\n";
	return true;
}
