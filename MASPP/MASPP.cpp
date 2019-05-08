#include <iostream>
#include"CDistance.h"
#include"CAgent.h"
#include <string>
#include <iomanip>
#define MAX_TOUR 1000

using namespace std;

bool mapftest(string testfile);
stMapf run_mapf(string path_g, string path_a);
stPoint** readpos_agent(string pathname, int& n);
bool chksolution(int init[], int goal[], int len, CGridMap* g);
int main() {
	//CGridMap g("../map/6x6map0.txt");
	//CDistance * d=new CDistance(&g);
	//d->printDistanceTable();
	//cout << "Done" << endl;
	mapftest("../tests/t6.test" );
	cout << "Done" << endl;
	return 0;
}

bool mapftest(string testfile) {
	string line;
	ifstream file(testfile);
	int pos = 0, total = 0;	// Number of test instances
	double avg_exp = 0;
	int max_exp = 0;
	int min_exp = INT_MAX;
	time_t max_t = 0;
	double avg_t = 0;
	double min_t = numeric_limits<double>::infinity();

	vector<stMapf> mapf_tests;

	if (!file.is_open()) return false;

	while (getline(file, line)) {
		if (line[0] == '#') continue;


		string grid_path = line;
		string agent_path;
		getline(file, agent_path);

		cerr << "Beginning instance " << total << endl;
		cout << "grid path= " + grid_path + "\t agent path= " + agent_path + "\n";

		stMapf tmp = run_mapf(grid_path, agent_path);
		mapf_tests.push_back(tmp);
		if (tmp.solved)
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
		if (it->solved) {
			cout << "Instance " << i++ << endl;
			cout << "\tGrid x=" << it->dim.x << ", y=" << it->dim.y << endl;
			cout << fixed;
			cout << "\tNum agents = " << it->num_agents << endl;
			cout << "\tTime = ";
			cout << setprecision(8) << it->time << "s\n";
			cout << "\tCollisions = " << it->collisions << endl;
			cout << "\tNum expansions = " << it->num_exp << endl;

			min_t = (it->time < min_t) ? it->time : min_t;
			max_t = (it->time > max_t) ? it->time : max_t;
			min_exp = (it->num_exp < min_exp) ? it->num_exp : min_exp;
			max_exp = (it->num_exp > max_exp) ? it->num_exp : max_exp;
			avg_exp += it->num_exp;
			avg_t += it->time;


			// Use cerr to print csv formatted data
			cerr << fixed;
			cerr << setprecision(3);
			cerr << i << "," << it->dim.x << "x" << it->dim.y << ",";
			cerr << it->num_agents << "," << (double)it->time << ",";
			cerr << it->collisions << "," << it->num_exp << ",";
			cerr << it->cost << endl;
		}
	}

	avg_exp = avg_exp / pos;
	avg_t = avg_t / pos;

	cout << "###############################################\n";
	cout << fixed;
	cout << setprecision(2);
	cout << "Average Node exp = " << avg_exp << endl;
	cout << "\tMin = " << min_exp << "\tMax = " << max_exp << endl;
	cout << "Average solution time = " << avg_t << endl;
	cout << "\tMin = " << min_t << "\tMax = " << max_t << endl;

	return true;
}
stMapf run_mapf(string path_g, string path_a) {

	int num_agents = -1;
	path_a = "../agents/" + path_a;
	stPoint** states = readpos_agent(path_a, num_agents);
	CGridMap grid("../grids/" + path_g);

	stMapf info(num_agents, 0, 0, false, grid.getDim());

	if (!chksolution((int*)states[0], (int*)states[1], num_agents, &grid))
		return info;

	Mapf m(num_agents, states[0], states[1], &grid);

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

	info.solved = true;
	info.num_exp = m.num_expansions();
	info.time = m.get_time();
	info.collisions = m.get_collisions();
	info.cost = m.cost();

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
		if (bfs.getsolncost_int() > MAX_TOUR) return false;
		cout << "Found path for " << i << endl;
	}
	cout << "Checked solutions OK\n";
	return true;
}
