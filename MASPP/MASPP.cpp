/*
梦开始的地方

程序的入口:
主函数

Written by Geoganlle Goo
**/

#include <iostream>
#include"CDistance.h"
#include"CMultiAgentSystem.h"
#include <string>
#include <iomanip>

using namespace std;

bool test_from_file(string testfile);//从文件中读取数据测试

stMultiAgentSystem run_multiagentsysystem(string path_g, string path_a);//运行一个测试实例(一张地图)
stPoint** readpos_agent(string pathname, int& n);//从文件中读取agent信息
//////////////////////////////////////////////////////////////////////////////////////////////////
void testState1() {//Pass
	stPoint p(1, 1);
	CState s(&p, 1);
	stMove m(SOUTH, 0);
	s.print_to_console();
	CState c(1, s, m);
	c.print_to_console();
}

void testSearch3() {
	int init[] = { 3,7,10,3 };
	int goal[] = { 0,6,3,1 };
	CGridMap g("../grids/t6_g1.txt");

	CSearch s(1, (stPoint*)init, (stPoint*)goal, &g);
	while (!s.expand());
}

void testSearch2() {
	stPoint p(1, 0);
	int init[] = { 0,0,1,0 };
	int goal[] = { 1,0,0,0 };
	stPoint* p_ptr = &p;
	CGridMap g("../map/4x4map0.txt");
	g.printGridMap();
	cout<<"can solved "<<chksolution(init, goal, 2, &g)<<endl;

	CSearch s(2, (stPoint*)init, (stPoint*)goal, &g);
	//std::cout << "";
	while (!s.expand());
	s.path(true);
}
 
void testSearch1() {
	int list[] = { 0, 4, 1, 1, 1, 4, 3, 3, 4, 0 };
	stPoint** plist = new stPoint * [5];
	for (int i = 0; i < 5; i++)
		plist[i] = (stPoint*)(list + 2 * i);
	for (int i = 0; i < 4; i++) {
		stPoint* p = plist[i];
		cout << "(" << p->x << "," << p->y << ")\n";
	}

	CGridMap g("../map/10x10map0.txt");
	cout<<g.passable(stPoint(0, 1))<<"\t";
	cout << g.passable(stPoint(9, 9))<<"\n";
	stPoint p1(0, 1);
	stPoint g1(9, 9);
	CSearch s(1, &p1, &g1, &g);
	cout << " 123\n";
	while (!s.expand());
	s.path(true);
}
//////////////////////////////////////////////////////////////////////////////////////////////////
int main() {

	testSearch2();
	//test_from_file("../tests/t6.test" );
	cout << "Done" << endl;
	return 0;
}

bool test_from_file(string testfile) {
	string line;
	ifstream file(testfile);
	int pos = 0, total = 0;	//测试数量
	double avg_exp = 0;
	int max_exp = 0;
	int min_exp = INT_MAX;
	time_t max_t = 0;
	double avg_t = 0;
	double min_t = numeric_limits<double>::infinity();

	vector<stMultiAgentSystem> tests_MAS_vector;

	if (!file.is_open()) return false;

	while (getline(file, line)) {
		if (line[0] == '#') continue;//允许输入注释

		string grid_path = line;//地图文件
		string agent_path;//智能体文件
		getline(file, agent_path);

		cerr << "Beginning instance " << total << endl;
		cout << "Grid path= " + grid_path + "\t Agent path= " + agent_path + "\n";

		stMultiAgentSystem tmp = run_multiagentsysystem(grid_path, agent_path);
		tests_MAS_vector.push_back(tmp);

		if (tmp.canbesolved_bool) {
			pos++;
		}
		else {
			cout << "Cannot solve instance " << total << endl;
		}
		total++;
	}

	file.close();

	cout << "###############################################\n";
	cout << "Solved " << pos << "/" << total << " instances\n";

	int i = 0;

	for (auto it = tests_MAS_vector.begin(); it != tests_MAS_vector.end(); it++) {
		if (it->canbesolved_bool) {
			cout << "Instance " << i++ << endl;
			cout << "GridDim  y = " << it->dim.y <<" x = " << it->dim.x << endl;
			cout << "Num agents = " << it->num_agents_int << endl;
			cout << "Time = ";
			cout << fixed<<setprecision(8) << it->time_timet << "s\n";
			cout << "Num Collisions = " << it->num_collisions_int << endl;
			cout << "Num expansions = " << it->num_expansions_int << endl;

			min_t = (it->time_timet < min_t) ? it->time_timet : min_t;
			max_t = (it->time_timet > max_t) ? it->time_timet : max_t;

			min_exp = (it->num_expansions_int < min_exp) ? it->num_expansions_int : min_exp;
			max_exp = (it->num_expansions_int > max_exp) ? it->num_expansions_int : max_exp;
			avg_exp += it->num_expansions_int;
			avg_t += it->time_timet;

			cout << endl;
		}
	}

	avg_exp = avg_exp / pos;
	avg_t = avg_t / pos;

	cout << "###############################################\n";
	cout << "Average Node exp = " << avg_exp << endl;
	cout << "\tMin = " << min_exp << "\tMax = " << max_exp << endl;
	cout << fixed << setprecision(4) << "Average solution time = " << avg_t << endl;
	cout << "\tMin = " << min_t << "\tMax = " << max_t << endl;

	return true;
}
stMultiAgentSystem run_multiagentsysystem(string path_g, string path_a) {

	int num_agents = -1;
	path_a = "../agents/" + path_a;
	stPoint** states = readpos_agent(path_a, num_agents);
	CGridMap grid("../grids/" + path_g);

	stMultiAgentSystem info(num_agents, 0, 0, 0, 0, false, grid.getDim());
	///智能体数量 扩展节点的数量 冲突的数量 解决方案总开销 解决方案总耗时 问题是否可以解决 地图尺寸

	if (!chksolution((int*)states[0], (int*)states[1], num_agents, &grid))
		return info;

	CMultiAgentSystem m(num_agents, states[0], states[1], &grid);
	//CMultiAgentSystem(int n, stPoint* s_init, stPoint* s_goal, CGridMap* gd);

	int res;
	while (res = m.resolve_conflicts()) {
		if (res == 2) {//节点过多
			delete[] states[0];
			delete[] states[1];
			delete[] states;
			cout << "Failed to solve! Node expand too much \n";
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
