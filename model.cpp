#include <iostream>  
#include <vector>  
#include <ctime>  
#include <cmath>
#include <cstdlib>  
#include <string>  
#include <algorithm>  
#include <fstream>
#include "gurobi_c++.h" 
#include "model.h"
#include "initialize.h"
#include "utility.h"
#include "config.h"
using namespace std;  

//build the gurobi environment
Model::Model(Nodes nodes, Vehicles amvs, string mol_name, TimeSpace_Network ts_network): node_set(nodes), amv_set(amvs), modelname(mol_name), network(ts_network)
{
    //start to calculate the optimization time
    start = clock(); // get current time
    mol_filename = "AMVs_CD_PF_" + mol_name;
    result_file = "../../results/result_" + modelname + ".dat";
    obj_dist = 0;
    unsv_pas = 0;
    unsv_fre = 0;
    target_val = 0;
    mavs_trip_duration = 0;
    //get tsarcs that come out or in the depot 
    depot_moveout = network.cap_moveout(0);
    depot_movein = network.cap_movein(0);
    out_depot_tsarcs = network.intersect_pos(network.get_tsarcs()[0], depot_moveout); //the indices of all arcs of depot_moveout in the complete set of moving arcs
    in_depot_tsarcs = network.intersect_pos(network.get_tsarcs()[0], depot_movein);  //the indices of all arcs of depot_movein in the complete set of moving arcs
}

void Model::write_model(GRBModel &mol)
{
    string mol_file = "../../lp_models/" + mol_filename + ".lp";
    mol.write(mol_file);  //write the model to file
}

void Model::run_model()
{
    string env_file = mol_filename + ".log";
    //create gurobi environment
    GRBEnv env = GRBEnv(true);
    env.set("LogFile", env_file);
    env.start();
    GRBModel model = GRBModel(env);
    add_variables(model);    //add variables
    add_objectives(model);   //add objectives
    add_constraints(model);  //add constraints
    write_model(model);  //write the model to file
    model.optimize();   //model optimization
    get_result(model);
}

double Model::get_totaldist()
{
    return obj_dist;
}

double Model::get_pas_unsv()
{
    return unsv_pas;
}

double Model::get_fre_unsv()
{
    return unsv_fre;
}

double Model::get_total_unsv()
{
    return unsv_pas + unsv_fre;
}

double Model::get_targetval()
{
    return target_val;
}