/*Coupling and Decoupling Tactics of Autonomous Modular Vehicles for Integrated Passenger and Freight Transport*/

#include <iostream>  
#include <vector>  
#include <ctime>  
#include <cmath>
#include <cstdlib>  
#include <string>  
#include <algorithm>  
#include <fstream>
#include "gurobi_c++.h"  
#include "utility.h"
#include "config.h"
#include "initialize.h"
#include "mainmodel.h"
#include "benchmark1.h"
#include "benchmark2.h"
#include "benchmark1_1.h"
using namespace std;  


int main()
{
    Nodes nodeset;    //customer node structure to store node information
    Vehicles amvset;   //vehicle structure to store vehicle information
    Initializer road_network("../../network_data/corridor4.txt"); 
    road_network.amv_struct(amvset);
    road_network.node_struct(nodeset, amvset.veh_type);
    TimeSpace_Network network1(nodeset.nodenum, nodeset.travel_tw, nodeset.service_tw, nodeset.nodetype, 
                                nodeset.traveltime_matrix, nodeset.service_time, nodeset.neighbours);

    //solve with gurobi
    try
    {
        MainModel mainmodel(nodeset, amvset, "m1", network1);
        // Benchmark1 benchmark1(nodeset, amvset, "b1", network1);
        Benchmark1_1 benchmark1_1(nodeset, amvset, "b1_1", network1);
        Benchmark2 benchmark2(nodeset, amvset, "b2", network1);
        mainmodel.run_model();
        benchmark2.run_model();
        benchmark1_1.run_model();
        // benchmark1.run_model();
    } catch ( GRBException e)
    {
        cout << " Error code = " << e.getErrorCode() << endl;
        cout << e.getMessage () << endl ;
    } catch (...)
    {
        cout << " Exception during optimization " << endl;
    }

    return 0;
}