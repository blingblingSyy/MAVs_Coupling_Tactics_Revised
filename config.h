// config.h

#ifndef _CONFIG_H_    //include guards
#define _CONFIG_H_

#include <vector>
using namespace std;

#define INF 0x3f3f3f3f     //infinite large number
#define SMALL 1e-4         //a small number

const int w1 = 50;   //Unit operational costs per travel distance per AMV
const int w2 = 1;   //Unit operational costs for AMV's total trip duration
const int w3 = 1000;  //Unit costs for each unserved request
const int w4 = 50;     //unit costs for passengers' total trip duration
const double fp = 0.8;   //Freight to passenger weight ratio of total unserved request

const int SPEED = 1;      //AMV speed (km/h)
const int MAX_DIST = 100;   //AMV maximum range (km)
const int VCAP_0 = 1;       //AMV capacity for passenger
const int VCAP_1 = 2;      //AMV capacity for freight
const int WAIT_0 = 1;       //waiting time limit for passenger-type AMV at each location
const int WAIT_1 = 3;       //waiting time limit for freight-type AMV at each location
const int WAIT_MAX_0 = 5;    //waiting time limit for passenger-type AMV at all locations
const int WAIT_MAX_1 = 7;    //waiting time limit for freight-type AMV at all locations
const int SERT_0 = 1;       //constant service time for passenger node
const int SERT_1 = 3;       //constant service time for freight node
const double SER_RATE_0 = 0.5;      //service rate for passenger node
const double SER_RATE_1 = 0.1;      //service rate for freight node
const int SER_PRD_0 = 10;   //service period for passenger node
const int SER_PRD_1 = 15;   //service period for freight node
const vector<int> DMR_0 = {1,12};   //demand distribution range for passenger node
const vector<int> DMR_1 = {1,60};   //demand distribution range for freight node
const double PAS_AMV_PROB = 0.5;    //ratio of passenger AMVs
const int VTYPE = 2;

//store nodes' information
struct Nodes    
{
    int nodenum;
    vector<int> nodetype;   //0 - passenger; 1 - freight
    vector<int> demands;
    vector<int> demand_type;    //0 - pickup; 1 - delivery
    vector<vector<int>> demand_pair;
    vector<vector<double>> initial_distance;
    vector<vector<double>> shortest_path;
    vector<vector<int>> traveltime_matrix;
    vector<vector<int>> traveltime_shortest;
    vector<int> service_time;
    vector<double> service_rate;
    vector<vector<int>> travel_tw;
    vector<vector<int>> service_tw;
    vector<vector<int>> match_amv;
    vector<vector<int>> neighbours;
};

//store AMVs' information
struct Vehicles     
{
    int veh_num;
    double veh_speed;
    int max_range;
    int max_plen;
    vector<int> veh_type;
    vector<int> veh_cap;
    vector<int> wait_lim;
};

//time space network
class TimeSpace_Network
{
    private:
        vector<vector<vector<int>>> tsnodes;
        vector<vector<vector<int>>> tsnodes_ser;
        vector<vector<vector<vector<int>>>> tsarcs;
        vector<vector<vector<int>>> tsarcs_move;
        vector<vector<vector<vector<int>>>> tsarcs_wait;
        vector<vector<vector<vector<int>>>> tsarcs_ser;
        vector<vector<vector<vector<int>>>> tsarcs_moveout;
        vector<vector<vector<vector<int>>>> tsarcs_movein;
    public:
        TimeSpace_Network(int node_num, vector<vector<int>> tvl_tw, vector<vector<int>> ser_tw, vector<int> nodetype,
                        vector<vector<int>> tvl_timemat, vector<int> serve_time, vector<vector<int>> adjacent_nodes);
        // TimeSpace_Network(Nodes& nodes);
        vector<vector<vector<int>>> get_tsnodes();
        vector<vector<vector<int>>> get_tsnodes_ser();  
        vector<int> get_sernodes_id_in_tsnodes(int cus_id);
        vector<vector<vector<vector<int>>>> get_tsarcs();   //0: moving arcs; 1: waiting arcs; 2: serving arcs
        vector<vector<vector<int>>> get_tsarcs_move();
        vector<vector<vector<int>>> get_tsarcs_wait();
        vector<vector<vector<int>>> get_tsarcs_ser(); 
        vector<vector<vector<int>>> cap_waitarc_cus(int cus_id);
        vector<vector<vector<int>>> cap_serarc_cus(int cus_id);
        vector<vector<vector<int>>> cap_moveout(int cus_id);
        vector<vector<vector<int>>> cap_movein(int cus_id);
        vector<vector<int>> cap_outflow(int cus_id, int t_id, int flow_type = 3);
        vector<vector<int>> cap_inflow(int cus_id, int t_id, int flow_type = 3);
        vector<vector<vector<int>>> intersect_arcs(vector<vector<vector<int>>> sort_arcset1, vector<vector<vector<int>>> sort_arcset2);
        vector<int> intersect_pos(vector<vector<vector<int>>> sort_arcset, vector<vector<vector<int>>> sort_arcsubset);
};

#endif
