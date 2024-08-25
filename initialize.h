// initialize.h

#ifndef _INITIALIZE_H_    //include guards
#define _INITIALIZE_H_

#include <vector>
using namespace std;

//Initialize the parameters for the model
class Initializer
{
    private: 
        const char* filename;
        int file_row;
        vector<vector<int>> data_vec;
        int amv_num;
        int pl_max;
        int plan_horizon;
        int node_num;
        vector<int> node_type;
        vector<int> dmd_type;
        vector<vector<double>> original_dist;
        void countrows();
        void readdata();
    public:
        Initializer(const char* filepath);
        int get_filerow();
        int get_amvnum();
        int get_plmax();
        int get_planhorizon();
        int get_nodenum();
        vector<int> get_nodetype();
        vector<int> get_dmdtype();
        vector<vector<double>> get_original_dist();
        void amv_struct(struct Vehicles &amvs);
        void node_struct(struct Nodes &nodes, vector<int> amv_type);
};

#endif
