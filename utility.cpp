#include <iostream>  
#include <vector>  
#include <ctime>  
#include <cstdlib>  
#include <string>  
#include <cstring>
#include <algorithm>  

#include "config.h"
#include "utility.h"

using namespace std;  


//generate random number with RandomNumber class
RandomNumber::RandomNumber() 
{
    srand((unsigned)time(NULL));
}

int RandomNumber::get_rint(int begin, int end) 
{
    return rand() % (end - begin + 1) + begin; 
}

float RandomNumber::get_rflt(int begin, int end)
{
    return begin + float(rand()) / float(RAND_MAX) * (end - begin); 
}

// multi-layer sorting from small to large
bool my_compare(vector<vector<int>> arc1, vector<vector<int>> arc2)
{
    if(arc1[0][0] != arc2[0][0]) 
        return arc1[0][0] < arc2[0][0];     //first layer compare
    else if(arc1[0][1] != arc2[0][1])
        return arc1[0][1] < arc2[0][1];     //second layer compare
    else if(arc1[1][0] != arc2[1][0])
        return arc1[1][0] < arc2[1][0];     //third layer compare
    else
        return arc1[1][1] < arc2[1][1];     //last layer compare
}

//generate random demand for each node according to their node type
vector<int> randdemand(vector<int> nodetype)  
{  
    vector<int> demand;
    demand.resize(int(nodetype.size()));
    RandomNumber r;
    for(int i = 0; i < nodetype.size(); i++)
    {
        switch(nodetype[i])
        {
            case 0:
                demand[i] = r.get_rint(DMR_0[0], DMR_0[1]);
                break;
            case 1:
                demand[i] = r.get_rint(DMR_1[0], DMR_1[1]);
                break;
            case 2: 
                demand[i] = 0;
                break;
        }
    }
    return demand;
}  

//generate demand pair (pickup demand, delivery demand) for each location
vector<vector<int>> get_dmdpair(vector<int> dmd_type, vector<int> demand)
{
    vector<vector<int>> dmd_pair(int(dmd_type.size()));
    for(int i = 0; i < dmd_type.size(); i++)
    {
        switch(dmd_type[i])
        {
            case 0:
                dmd_pair[i] = {demand[i], 0};
                break;
            case 1:
                dmd_pair[i] = {0, demand[i]};
                break;
        }
    }
    return dmd_pair;
}

//generate service time length for each location
vector<int> get_st(vector<int> node_type)
{
    vector<int> serve_time(int(node_type.size()));
    for(int i = 0; i < node_type.size(); i++)
    {
        switch(node_type[i])
        {
            case 0: 
                serve_time[i] = SERT_0;
                break;
            case 1:
                serve_time[i] = SERT_1;
                break;
            case 2: 
                serve_time[i] = 0;
                break;
        }
    }
    return serve_time;
}

//generate service rate for each customer
vector<double> get_rate(vector<int> node_type)
{
    vector<double> serve_rate(int(node_type.size()));
    for(int i = 0; i < node_type.size(); i++)
    {
        switch(node_type[i])
        {
            case 0: 
                serve_rate[i] = SER_RATE_0;
                break;
            case 1:
                serve_rate[i] = SER_RATE_1;
                break;
            case 2: 
                serve_rate[i] = 0;
                break;
        }
    }
    return serve_rate;
}

double get_amvprob(vector<int> nodetype)
{
    double sum = 0, zero_num = 0;
    for(int i = 0; i < nodetype.size(); i++)
    {
        if(nodetype[i] != 2)
        {
            sum += 1;
            zero_num += (1-nodetype[i]);
        }
    }
    return zero_num / sum;
}

//generate type for each AMV - 0 for passenger and 1 for freight
vector<int> get_amvtype(int amv_num, double prob)   //prob is for passenger AMVs
{
    vector<int> amv_type(amv_num);
    int pas_amvs = int(ceil(amv_num * prob));
    if(prob == 0)
    {
        for(int i = 0; i < amv_num; i++)
        {
            amv_type[i] = 1;
        }
    }
    else if(prob == 1)
    {
        for(int i = 0; i < amv_num; i++)
        {
            amv_type[i] = 0;
        }        
    }
    else
    {
        for(int i = 0; i < pas_amvs; i++)
        {
            amv_type[i] = 0;
        }
        for(int i = pas_amvs; i < amv_num; i++)
        {
            amv_type[i] = 1;
        }
    }
    return amv_type;
    // vector<int> amv_type(amv_num);
    // RandomNumber r;
    // for(int i = 0; i < amv_num; i++)  
    // {  
    //     amv_type[i] = (r.get_rflt() <= prob) ? 0 : 1;  
    // }  
}

//link vehicle capacity with each vehicle type
vector<int> get_amvcap(vector<int> amv_type)    //0 for passenger and 1 for freight
{
    vector<int> amv_cap(int(amv_type.size()));
    for(int i = 0; i < amv_type.size(); i++)
    {
        amv_cap[i] = (amv_type[i] == 0) ? VCAP_0: VCAP_1;
    }
    return amv_cap;
}

//generate waiting time limit for each AMV
vector<int> get_amvwlim(vector<int> amv_type)
{
    vector<int> amv_wlim(int(amv_type.size()));
    for(int i = 0; i < amv_type.size(); i++)
    {
        amv_wlim[i] = (amv_type[i] == 0) ? WAIT_0: WAIT_1;
    }
    return amv_wlim;    
}

//calculate the distance reduction factor for a platoon of specific length
double pl_factor(int length)
{
    if(length <= 0)
    {
        throw "Division by zero or negative condition!";
    }
    return double(1+0.90*(length-1)) / double(length);
}

//get the combination of customers and amvs of the same type
vector<vector<int>> get_matchamvs(vector<int> nodetype, vector<int> amvtype)
{
    vector<vector<int>> match_com;
    match_com.resize(int(nodetype.size()));
    for(int i = 0; i < nodetype.size(); i++)
    {
        if(nodetype[i] < 2)
        {
            for(int v = 0; v < amvtype.size(); v++)
            {
                if(amvtype[v] == nodetype[i])
                {
                    match_com[i].push_back(v);
                }
            }
        }
        else    //nodetype[i] == 2
        {
            for(int v = 0; v < amvtype.size(); v++)
            {
                match_com[i].push_back(v);
            }            
        }
    }
    return match_com;
}

//find the neighbours of each node
vector<vector<int>> get_neighbours(vector<vector<double>> init_dist, int node_num)
{
    vector<vector<int>> adjacent_nodes(node_num);
    for(int i = 0; i < node_num; i++)
    {
        for(int j = 0; j < node_num; j++)
        {
            if((i != j) && (init_dist[i][j] != INF))
            {
                adjacent_nodes[i].push_back(j);
            }
        }
    }
    return adjacent_nodes;
}

//use Dijkstra algorithm to calculate the shortest path distance matrix
vector<vector<double>> Dijkstra(vector<vector<double>> init_dist, int node_num)
{
    //initial distance that has not been determined yet, but will be updated with the algorithm
	vector<vector<double>> dist_matrix;
	dist_matrix.resize(node_num);
	for(int i = 0; i < node_num; i++)
	{
		dist_matrix[i].resize(node_num, INF);
		dist_matrix[i][i] = 0;

		//unvisited node
		vector<int> unvisited_nodes(node_num);
		for(int j = 0; j < node_num; j++)
		{
			unvisited_nodes[j] = j;
		}
		
		auto compare = [&](int s, int r) {return dist_matrix[i][s] < dist_matrix[i][r];};
		
		//the main loop
		while(!unvisited_nodes.empty())
		{
			vector<int>::iterator iter = min_element(unvisited_nodes.begin(),unvisited_nodes.end(), compare);
			auto min_pos = iter - unvisited_nodes.begin();
			int min_val = unvisited_nodes[min_pos];
			unvisited_nodes.erase(iter);

			for(int k = 0; k < unvisited_nodes.size(); k++)
			{
				double alt_dist = dist_matrix[i][min_val] + init_dist[min_val][unvisited_nodes[k]];
				if(alt_dist < dist_matrix[i][unvisited_nodes[k]])
				{
					dist_matrix[i][unvisited_nodes[k]] = alt_dist;
				}
			}
		}
	}
    return dist_matrix;
}

//generate the travel time matrix based on original distance matrix
vector<vector<int>> get_tvltime(vector<vector<double>> init_dist, int node_num)
{
    vector<vector<int>> travel_time(node_num);
    for(int i = 0; i < node_num; i++)
    {
        travel_time[i].resize(node_num);
        for(int j = 0; j < node_num; j++)
        {
            travel_time[i][j] = (init_dist[i][j] < INF) ? int(init_dist[i][j] / SPEED) : INF; //revise later: rounding to integer; //int(init_dist[i][j] / SPEED * 60)
        }
    }
    return travel_time;
}

//generate available AMV travelling time windows for each customer according to their node type
//to model time-space network, transform time into integers
vector<vector<int>> get_tvltw(vector<double> source_dist, int plan_horizon)   //source dist is the shortest path distance from the depot
{
    vector<vector<int>> tvl_tw(int(source_dist.size()));
    for(int i = 0; i < source_dist.size(); i++)
    {
        int source_time = int(source_dist[i] / SPEED); //int(source_dist[i] / SPEED * 60);
        if(plan_horizon - source_time <= 1 + source_time)
        {
            cerr << "Infeasible travel time window of node "<< i << endl;
        }
        else
            tvl_tw[i] = {1 + source_time, plan_horizon - source_time};
    }
	return tvl_tw;
}

//generate available AMV serving time window for each customer according to their node type
vector<vector<int>> get_sertw(vector<vector<int>> tvl_tw, vector<int> nodetype) //the two parameters should be of the same length
{
    RandomNumber r;
    vector<vector<int>> ser_tw(int(tvl_tw.size()));
    for(int i = 0; i < tvl_tw.size(); i++)
    {
        if(nodetype[i] != 2)
        {
            int serve_period = (nodetype[i] == 0) ? SER_PRD_0 : SER_PRD_1;
            if(tvl_tw[i][1] - serve_period <= tvl_tw[i][0])
            {
                ser_tw[i] = tvl_tw[i];
            }
            else
            {
                int ser_start = r.get_rint(tvl_tw[i][0], tvl_tw[i][1] - serve_period);
                int ser_end = ser_start + serve_period;  //the service time window represents the time intervals allowable to start and end service, this is different from the service time windows in algorithms
                ser_tw[i] = {max(ser_start, tvl_tw[i][0]), min(ser_end, tvl_tw[i][1])};
            }
        }
        else
        {
            ser_tw[i] = tvl_tw[i];
        }
    }
    return ser_tw;
}

void calibrate_sertw(vector<vector<int>>& initial_sertw, vector<int> servetime, vector<vector<int>> tvl_tw)
{
    // assert(initial_sertw.size() == tvl_tw.size());
    for(int i = 0; i < initial_sertw.size(); i++)
    {
        initial_sertw[i][0] = max(initial_sertw[i][0], tvl_tw[i][0]);
        initial_sertw[i][1] = min(initial_sertw[i][1], tvl_tw[i][1] - servetime[i]);
    }
}


//initialize the nodes in the timespace network
TimeSpace_Network::TimeSpace_Network(int node_num, vector<vector<int>> tvl_tw, vector<vector<int>> ser_tw, vector<int> nodetype,
                                    vector<vector<int>> tvl_timemat, vector<int> serve_time, vector<vector<int>> adjacent_nodes)
{
    //tsnodes, tsnodes_cus, tsnodes_ser
    tsnodes.resize(node_num);
    tsnodes_ser.resize(node_num);
    for(int i = 0; i < node_num; i++)
    {
        for(int t = tvl_tw[i][0]; t <= tvl_tw[i][1]; t++)
        {
            tsnodes[i].push_back({i,t});
        }
        for(int t = ser_tw[i][0]; t <= ser_tw[i][1]; t++)
        {
            tsnodes_ser[i].push_back({i,t});
        }
    }
    //tsnodes_cus.insert(tsnodes_cus.end(), tsnodes.begin()+1, tsnodes.end()-1);

    //tsarcs_move, tsarcs_moveout, tsarcs_movein
    tsarcs_moveout.resize(node_num);
    tsarcs_movein.resize(node_num);
    for(int i = 0; i < tsnodes.size(); i++) // 0 ~ nodenum-1
    {
        for(int t = 0; t < tsnodes[i].size(); t++)
        {
            int ti = tsnodes[i][t][1];
            for(int k = 0; k < adjacent_nodes[i].size(); k++)
            {
                int j = adjacent_nodes[i][k];
                int tj = ti + tvl_timemat[i][j];
                if((tj >= tvl_tw[j][0]) && (tj <= tvl_tw[j][1]))
                {
                    tsarcs_move.push_back({{i, ti}, {j, tj}});
                    tsarcs_moveout[i].push_back({{i, ti}, {j, tj}});
                    tsarcs_movein[j].push_back({{i, ti}, {j, tj}});
                }
            }
        }
    }
    sort(tsarcs_move.begin(), tsarcs_move.end(), my_compare);
    for(int i = 0; i < node_num; i++)
    {
        sort(tsarcs_moveout[i].begin(), tsarcs_moveout[i].end(), my_compare);
        sort(tsarcs_movein[i].begin(), tsarcs_movein[i].end(), my_compare);
    }

    //tsarcs_wait
    tsarcs_wait.resize(node_num);   
    for(int i = 0; i < tsnodes.size(); i++)
    {
        for(int t = 0; t < tsnodes[i].size()-1; t++)
        {
            tsarcs_wait[i].push_back({tsnodes[i][t], tsnodes[i][t+1]});
        }
    }

    //tsarcs_serve
    tsarcs_ser.resize(node_num);
    //tsarcs_ser[0].push_back({tsnodes_ser[0][0], tsnodes_ser[0][0]});
    for(int i = 1; i < tsnodes_ser.size(); i++)
    {
        if(nodetype[i] != 2)
        {
            //! case: the service time is constant
            for(int t = 0; t < tsnodes_ser[i].size(); t++)
            {
                int start_t = tsnodes_ser[i][t][1];
                int end_t = start_t + serve_time[i];
                tsarcs_ser[i].push_back({tsnodes_ser[i][t], {i, end_t}});   //length of service time at the depot is 0
            }
            //! case: the service time is variable and dependent on the service amount of a node
            // for(int t = 0; t < tsnodes_ser[i].size()-1; t++)
            // {
            //     int time_i = tsnodes_ser[i][t][1];
            //     int remain_time = ser_tw[i][1] - time_i;
            //     for(int k = 1; k <= remain_time; k++)
            //     {
            //         tsarcs_ser[i].push_back({tsnodes_ser[i][t], tsnodes_ser[i][t+k]});   //length of service time at the depot is 0
            //     }
            // }
        }
        // else
        // {
        //     tsarcs_ser[i].push_back({tsnodes_ser[i][0], tsnodes_ser[i][0]});
        // }
    }

    //tsarcs
    tsarcs.resize(3);   //0: moving arcs; 1: waiting arcs; 2: serving arcs
    tsarcs[0] = tsarcs_move;
    for(int i = 1; i < tsarcs_wait.size(); i++)
    {
        tsarcs[1].insert(tsarcs[1].end(), tsarcs_wait[i].begin(), tsarcs_wait[i].end());
    }
    for(int i = 1; i < tsarcs_ser.size(); i++)
    {
        if(nodetype[i] != 2)
        {
            tsarcs[2].insert(tsarcs[2].end(), tsarcs_ser[i].begin(), tsarcs_ser[i].end());
        }
    }
}

//return the set of all tsnodes
vector<vector<vector<int>>> TimeSpace_Network::get_tsnodes()
{
    return tsnodes;
}

//return the set of index of all tsnodes_ser
vector<vector<vector<int>>> TimeSpace_Network::get_tsnodes_ser()    //depot and intersections have no service nodes
{
    return tsnodes_ser;
}

//obtain the corresponding indices in the set of time-space nodes (tsnodes) for a customer's service time-space nodes (tsnodes_ser)
vector<int> TimeSpace_Network::get_sernodes_id_in_tsnodes(int cus_id)
{
    vector<int> sertime_idx;
    for(int t = 0; t < tsnodes_ser[cus_id].size(); t++)
    {
        int serve_ti = tsnodes_ser[cus_id][t][1];
        int st_idx = serve_ti - tsnodes[cus_id][0][1];
        sertime_idx.push_back(st_idx);
    }
    return sertime_idx;
}

//return the set of tsarcs
vector<vector<vector<vector<int>>>> TimeSpace_Network::get_tsarcs()
{
    return tsarcs;
}

//return the set of tsarcs_move
vector<vector<vector<int>>> TimeSpace_Network::get_tsarcs_move()
{
    return tsarcs[0];
}

//return the set of all tsarcs_wait
vector<vector<vector<int>>> TimeSpace_Network::get_tsarcs_wait()
{
    return tsarcs[1];
}

//return the set of tsarcs_ser
vector<vector<vector<int>>> TimeSpace_Network::get_tsarcs_ser()
{
    return tsarcs[2];
}

//capture the set of tsarcs_wait given a location
vector<vector<vector<int>>> TimeSpace_Network::cap_waitarc_cus(int cus_id)
{
    return tsarcs_wait[cus_id];
}

//capture the set of tsarcs_ser given a location
vector<vector<vector<int>>> TimeSpace_Network::cap_serarc_cus(int cus_id)
{
    return tsarcs_ser[cus_id];
}

//capture the outflow moving arcs from a tsnode
vector<vector<vector<int>>> TimeSpace_Network::cap_moveout(int cus_id)
{
    return tsarcs_moveout[cus_id];
}

//capture the inflow moving arcs from a tsnode
vector<vector<vector<int>>> TimeSpace_Network::cap_movein(int cus_id)
{
    return tsarcs_movein[cus_id];
}

//capture the outflow arcs from a tsnode 
vector<vector<int>> TimeSpace_Network::cap_outflow(int cus_id, int t_id, int flow_type)
{
    vector<vector<int>> outid_set;
    vector<int> out_node = {cus_id, t_id};
    if(flow_type == 3)
    {
        for(int a = 0; a < tsarcs.size(); a++)
        {
            for(int i = 0; i < tsarcs[a].size(); i++)
            {
                if(tsarcs[a][i][0] == out_node)
                {
                    outid_set.push_back({a,i}); //{arc_type, arc_index}
                }
            }
        }        
    }
    else
    {
        for(int i = 0; i < tsarcs[flow_type].size(); i++)
        {
            if(tsarcs[flow_type][i][0] == out_node)
            {
                outid_set.push_back({flow_type, i});
            }
        }        
    }
    return outid_set;
}

//capture the inflow arcs from a tsnode
vector<vector<int>> TimeSpace_Network::cap_inflow(int cus_id, int t_id, int flow_type)
{
    vector<vector<int>> inid_set;
    vector<int> in_node = {cus_id, t_id};
    if(flow_type == 3)
    {
        for(int a = 0; a < tsarcs.size(); a++)
        {
            for(int i = 0; i < tsarcs[a].size(); i++)
            {
                if(tsarcs[a][i][1] == in_node)
                {
                    inid_set.push_back({a,i});
                }
            }
        }       
    }
    else
    {
        for(int i = 0; i < tsarcs[flow_type].size(); i++)
        {
            if(tsarcs[flow_type][i][1] == in_node)
            {
                inid_set.push_back({flow_type, i});
            }
        }        
    }
    return inid_set;
}

//find the common set of two arc set
vector<vector<vector<int>>> TimeSpace_Network::intersect_arcs(vector<vector<vector<int>>> sort_arcset1, vector<vector<vector<int>>> sort_arcset2)
{
    vector<vector<vector<int>>> common_set;
    set_intersection(sort_arcset1.begin(), sort_arcset1.end(), sort_arcset2.begin(), sort_arcset2.end(), back_inserter(common_set));
    return common_set;
}

//find the position of elements of vector 2 in vector 1
vector<int> TimeSpace_Network::intersect_pos(vector<vector<vector<int>>> sort_arcset, vector<vector<vector<int>>> sort_arcsubset)
{
    vector<int> pos_in_arc1;
    vector<vector<vector<int>>>::iterator first1 = sort_arcset.begin();
    vector<vector<vector<int>>>::iterator last1 = sort_arcset.end();
    vector<vector<vector<int>>>::iterator first2 = sort_arcsubset.begin();
    vector<vector<vector<int>>>::iterator last2 = sort_arcsubset.end();    
    while(first1 != last1 && first2 != last2)
    {
        if(*first1 < *first2)
            ++first1;
        else
        {
            if(!(*first2 < *first1)) //*first2 == *first1
            {
                pos_in_arc1.push_back(static_cast<int>(first1 - sort_arcset.begin()));
                ++first1;
            }
            ++first2;
        }
    }
    return pos_in_arc1;
}