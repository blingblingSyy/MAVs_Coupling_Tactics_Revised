#include <iostream>  
#include <vector>  
#include <ctime>  
#include <cmath>
#include <cstdlib>  
#include <string>  
#include <algorithm>  
#include <fstream>
#include "initialize.h"
#include "utility.h"
#include "config.h"
using namespace std;  

//initialize the data in the file
Initializer::Initializer(const char* filepath)
{
    filename = filepath;
    file_row = 1;
    countrows();
    readdata();
    amv_num = data_vec[0][0];
    pl_max = data_vec[0][1];
    plan_horizon = data_vec[1][0];
    node_type = data_vec[3];
    dmd_type = data_vec[4];
}

//count the rows in the file
void Initializer::countrows()
{
    ifstream inputstr(filename);
	char chr;
	while(inputstr.get(chr))
	{
		if(chr == '\n')
			file_row++;
	}
	inputstr.close();
}

//read data from data source file
void Initializer::readdata()
{
	vector<int> raw_data;
	vector<int>::iterator it;
	ifstream file(filename);    //const char* filename = "corridor.txt";
	
	int num;
	while(file >> num)
	{
		raw_data.push_back(num);
	}
	file.close();

    it = raw_data.begin();
    data_vec.resize(file_row-5);
    while (it != raw_data.end())
    {
       for(int i = 0; i < file_row; i++)
        {
            if(i == 0)   //i = 0
            {
                data_vec[i].resize(2);
                for(int j = 0; j < 2; j++)
                {
                    data_vec[i][j] = *it;
                    it++;
                }
            }
            else if(i < 3)  //i = 1,2
            {
                data_vec[i].resize(1);
                data_vec[i][0] = *it;
                it++;
                if(i > 1)   //i = 2
                {
                    node_num = data_vec[2][0]; //! number of nodes
                    original_dist.resize(node_num);
                }
            }
            else if(i < 5)   //i = 3,4
            {
                data_vec[i].resize(node_num);
                for(int j = 0; j < node_num; j++)
                {
                    data_vec[i][j] = *it;
                    it++;
                }
            }
            else if(i >= 5) //i = 5,...,node_num+5-1
            {
                original_dist[i-5].resize(node_num);
                for(int j = 0; j < node_num; j++)
                {
                    if((*it) == -1)
                        original_dist[i-5][j] = INF;
                    else
                        original_dist[i-5][j] = double(*it);
                    it++;
                }
            }
        }
    }
}

//get the number of rows in the file
int Initializer::get_filerow()
{
    return file_row;
};

//get the number of amvs in the road network
int Initializer::get_amvnum()
{
    return amv_num;
};

//get the maximum platoon length set in the road network
int Initializer::get_plmax()
{
    return pl_max;
};

//get the planning horizon in the road network
int Initializer::get_planhorizon()
{
    return plan_horizon;
};

//get the number of nodes in the road network
int Initializer::get_nodenum()
{
    return node_num;
};

//get the passenger or freight types of each node in the road network
vector<int> Initializer::get_nodetype()
{
    return node_type;
};

//get the demand types of each node in the road network
vector<int> Initializer::get_dmdtype()
{
    return dmd_type;
};

//get the original distance matrix from the road network
vector<vector<double>> Initializer::get_original_dist()
{
    return original_dist;
}

//initialize the amv structure
void Initializer::amv_struct(struct Vehicles &amvs)
{
    //initialize the information of amv_set
    amvs.veh_num = amv_num;
    amvs.veh_speed = SPEED;
    amvs.max_range = MAX_DIST;
    amvs.max_plen = pl_max;
    amvs.veh_type = {0,0,0,1,1,1}; //get_amvtype(amv_num, get_amvprob(node_type));
    amvs.veh_cap = {1,1,1,2,2,2}; //get_amvcap(amvs.veh_type);
    amvs.wait_lim = get_amvwlim(amvs.veh_type);
}

//initialize the node structure
void Initializer::node_struct(struct Nodes &nodes, vector<int> amv_type)
{
    nodes.nodenum = node_num;
    nodes.nodetype = node_type;
    nodes.demands = {0,2,0,1,2,1};//randdemand(node_type);  //{0,50,7,36,11,0}
    nodes.demand_type = dmd_type;
    nodes.demand_pair = get_dmdpair(dmd_type, nodes.demands);
    nodes.service_time = get_st(node_type);
    nodes.service_rate = get_rate(node_type);
    nodes.initial_distance = original_dist;
    nodes.shortest_path = Dijkstra(original_dist, node_num);
    nodes.traveltime_matrix = get_tvltime(original_dist, node_num);
    nodes.traveltime_shortest = get_tvltime(nodes.shortest_path, node_num);
    nodes.travel_tw = get_tvltw(nodes.shortest_path[0], plan_horizon);
    nodes.service_tw = {{1,30}, {8,20}, {1,30}, {9,16}, {20,28}, {15,18}};//get_sertw(nodes.travel_tw, node_type);   //{{1,50}, {30,45}, {21,31}, {32,47}, {33,43}, {2,49}}
    calibrate_sertw(nodes.service_tw, nodes.service_time, nodes.travel_tw);
    nodes.neighbours = get_neighbours(original_dist, node_num);
    nodes.match_amv = get_matchamvs(node_type, amv_type);
}