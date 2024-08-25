// utility.h

#ifndef _UTILITY_H_    //include guards
#define _UTILITY_H_

#include <vector>
using namespace std;

class RandomNumber  
{  
     public:  
          RandomNumber();
          int get_rint(int begin = 0, int end = 1);
          float get_rflt(int begin = 0, int end = 1);
}; 
vector<int> randdemand(vector<int> nodetype);
bool my_compare(vector<vector<int>> arc1, vector<vector<int>> arc2);
vector<vector<int>> get_dmdpair(vector<int> dmd_type, vector<int> demand);
vector<int> get_st(vector<int> node_type);
vector<double> get_rate(vector<int> node_type);
double get_amvprob(vector<int> nodetype);
vector<int> get_amvtype(int amv_num, double prob);
vector<int> get_amvcap(vector<int> amv_type);
vector<int> get_amvwlim(vector<int> amv_type);
double pl_factor(int length);
vector<vector<int>> get_matchamvs(vector<int> nodetype, vector<int> amvtype);
vector<vector<int>> get_neighbours(vector<vector<double>> init_dist, int node_num);
vector<vector<double>> Dijkstra(vector<vector<double>> init_dist, int node_num);
vector<vector<int>> get_tvltime(vector<vector<double>> init_dist, int node_num);
vector<vector<int>> get_tvltw(vector<double> source_dist, int plan_horizon);
vector<vector<int>> get_sertw(vector<vector<int>> tvl_tw, vector<int> nodetype);
void calibrate_sertw(vector<vector<int>>& initial_sertw, vector<int> servetime, vector<vector<int>> tvl_tw);

#endif
