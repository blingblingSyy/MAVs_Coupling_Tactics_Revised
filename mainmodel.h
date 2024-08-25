// mainmodel.h

#ifndef _MAINMODEL_H_    //include guards
#define _MAINMODEL_H_

#include <vector>
#include <fstream>
#include <string>
#include "gurobi_c++.h" 
#include "model.h"
#include "config.h"
using namespace std;

//Initialize the model
class MainModel: public Model
{
    protected:
        virtual void add_variables(GRBModel &mol);
        virtual void add_objectives(GRBModel &mol);
        virtual void add_constraints(GRBModel &mol);
        virtual void get_result(GRBModel &mol);
        vector<vector<GRBVar>> x;
        vector<vector<vector<GRBVar>>> z;
        vector<vector<vector<GRBVar>>> y;
        vector<vector<vector<GRBVar>>> E;
        vector<vector<vector<GRBVar>>> e;
    public:
        MainModel(Nodes nodes, Vehicles amvs, string modelname, TimeSpace_Network ts_network);
};

#endif
