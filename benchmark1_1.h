// benchmark1_1.h

#ifndef _BENCHMARK1_1_H_    //include guards
#define _BENCHMARK1_1_H_

#include <vector>
#include <fstream>
#include "gurobi_c++.h" 
#include "config.h"
#include "model.h"
using namespace std;

//Initialize the model
class Benchmark1_1: public Model
{
    protected:
        virtual void add_variables(GRBModel &mol);
        virtual void add_objectives(GRBModel &mol);
        virtual void add_constraints(GRBModel &mol);
        virtual void get_result(GRBModel &mol);
        vector<vector<GRBVar>> x;
        vector<vector<vector<GRBVar>>> z;
        vector<vector<vector<vector<GRBVar>>>> E;
        vector<vector<vector<GRBVar>>> e;
        vector<vector<vector<GRBVar>>> y;
        vector<vector<GRBVar>> g;
        vector<vector<vector<vector<GRBVar>>>> F;
        vector<int> mav_num_type;
    public:
        Benchmark1_1(Nodes nodes, Vehicles amvs, string modelname, TimeSpace_Network ts_network);
};


#endif
