// model.h

#ifndef _MODEL_H_    //include guards
#define _MODEL_H_

#include <vector>
#include <fstream>
#include "gurobi_c++.h" 
#include "config.h"
#include "string"
using namespace std;

//Initialize the model
class Model
{
    protected:
        Nodes node_set;
        Vehicles amv_set;
        TimeSpace_Network network;
        string modelname;
        string mol_filename;
        string result_file;
        virtual void add_variables(GRBModel &mol) = 0;
        virtual void add_objectives(GRBModel &mol) = 0;
        virtual void add_constraints(GRBModel &mol) = 0;
        virtual void get_result(GRBModel &mol) = 0;
        void write_model(GRBModel &mol);
        vector<vector<vector<int>>> depot_moveout;
        vector<vector<vector<int>>> depot_movein;
        vector<int> out_depot_tsarcs;
        vector<int> in_depot_tsarcs;
        double obj_dist;
        double unsv_pas;
        double unsv_fre;
        double target_val;
        double mavs_trip_duration;
        double pas_trip_duration;
        clock_t start;
        int optimstatus;
        double duration;
        ofstream outfile;
    public:
        Model(Nodes nodes, Vehicles amvs, string mol_name, TimeSpace_Network network);
        void run_model();
        double get_totaldist();
        double get_pas_unsv();
        double get_fre_unsv();
        double get_total_unsv();
        double get_targetval();
};

#endif
