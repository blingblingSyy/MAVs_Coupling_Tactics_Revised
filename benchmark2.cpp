#include <iostream>  
#include <vector>  
#include <ctime>  
#include <cmath>
#include <cstdlib>  
#include <string>  
#include <algorithm>  
#include <fstream>
#include <numeric>
#include "gurobi_c++.h" 
#include "benchmark2.h"
#include "initialize.h"
#include "utility.h"
#include "config.h"
using namespace std;  


//build the gurobi environment
Benchmark2::Benchmark2(Nodes nodes, Vehicles amvs, string modelname, TimeSpace_Network ts_network): Model(nodes, amvs, modelname, ts_network)
{

}

void Benchmark2::add_variables(GRBModel &mol)
{
    //x_{i,v}
    x.resize(node_set.nodenum);
    for(int i = 1; i < node_set.nodenum; i++)   //add constraint: the depot and intersections will not be served
    {  
        x[i].resize(amv_set.veh_num);
        for(int v = 0; v < amv_set.veh_num; v++)
        {
            x[i][v] = mol.addVar(0.0, 1.0, 0, GRB_BINARY, "x" + to_string(i) + "," + to_string(v));
        }
    }   

    //z_{l,v}, y_{l,h}, E_{l,v}
    auto arc_type = network.get_tsarcs().size();
    z.resize(arc_type);
    E.resize(arc_type);
    for(int a = 0; a < arc_type; a++)
    {
        auto arc_size = network.get_tsarcs()[a].size();
        z[a].resize(arc_size); //! if a == 2, already exclude the serving arcs for the nodes with nodetype 2
        E[a].resize(arc_size);
        for(int L = 0; L < arc_size; L++)
        {
            int s0 = network.get_tsarcs()[a][L][0][0];
            int t0 = network.get_tsarcs()[a][L][0][1];
            int s1 = network.get_tsarcs()[a][L][1][0];
            int t1 = network.get_tsarcs()[a][L][1][1];
            z[a][L].resize(amv_set.veh_num);
            E[a][L].resize(amv_set.veh_num);
            for(int v = 0; v < amv_set.veh_num; v++)
            {
                z[a][L][v] = mol.addVar(0.0, 1.0, 0, GRB_BINARY, 
                        "z" + to_string(a) + ",(" + to_string(s0) + "," + to_string(t0) + "," + to_string(s1) + "," + to_string(t1)+ ")," + to_string(v));
                E[a][L][v] = mol.addVar(0.0, INFINITY, 0, GRB_CONTINUOUS, 
                        "E" + to_string(a) + ",(" + to_string(s0) + "," + to_string(t0) + "," + to_string(s1) + "," + to_string(t1)+ ")," + to_string(v));                                
            }
        }
    }

    //e_{(i,t),v}  //! only build on the tsnodes_ser and the subset of matching mavs
    e.resize(node_set.nodenum);
    for(int i = 1; i < node_set.nodenum; i++)
    {
        e[i].resize(network.get_tsnodes()[i].size());
        for(int t = 0; t < network.get_tsnodes()[i].size(); t++)    //t = actual time - travel_tw[i][0]
        {
            int time_i = network.get_tsnodes()[i][t][1];
            e[i][t].resize(amv_set.veh_num);
            for(int v = 0; v < amv_set.veh_num; v++)
            {
                e[i][t][v] = mol.addVar(0.0, GRB_INFINITY, 0, GRB_INTEGER, "e(" + to_string(i) + "," + to_string(time_i) + ")," + to_string(v));
            }
        }
    }

    mol.update();
}

void Benchmark2::add_objectives(GRBModel &mol)
{
    //add objective to the model
    GRBQuadExpr obj = 0.0;
    
    //1st term: total distance
    for(int L = 0; L < network.get_tsarcs()[0].size(); L++)  
    {  
        int s0 = network.get_tsarcs()[0][L][0][0];
        int s1 = network.get_tsarcs()[0][L][1][0];
        for(int v = 0; v < amv_set.veh_num; v++)  
        {    
            obj += w1 * node_set.initial_distance[s0][s1] * z[0][L][v]; //! 0: arctype -> move
        }
    }

    // 2nd term: total trip duration
    for(int v = 0; v < amv_set.veh_num; v++)
    {
        GRBLinExpr in_time = 0.0, out_time = 0.0;
        for(int L = 0; L < in_depot_tsarcs.size(); L++)
        {
            int t1 = network.get_tsarcs()[0][in_depot_tsarcs[L]][1][1];            
            in_time += z[0][in_depot_tsarcs[L]][v] * t1;
        }
        for(int L = 0; L < out_depot_tsarcs.size(); L++)
        {
            int t0 = network.get_tsarcs()[0][out_depot_tsarcs[L]][0][1];          
            out_time += z[0][out_depot_tsarcs[L]][v] * t0;
        }
        obj += w2 * (in_time - out_time);
    }

    // //3rd term: unserved demands
    // GRBLinExpr pas_unserved = 0.0, fre_unserved = 0.0;
    // for(int i = 1; i < node_set.nodenum; i++)
    // {
    //     if(node_set.nodetype[i] == 0) pas_unserved += 1;
    //     else if (node_set.nodetype[i] == 1) fre_unserved += 1;
    // }
    // for(int i = 1; i < node_set.nodenum; i++)   //all time-space service nodes
    // {  
    //     if(node_set.nodetype[i] != 2)
    //     {
    //         for(int r = 0; r < node_set.match_amv[i].size(); r++)    //node_set.match_amv[i]
    //         {
    //             switch(node_set.nodetype[i])
    //             {
    //                 case 0:
    //                     pas_unserved -= x[i][node_set.match_amv[i][r]];
    //                     break;
    //                 case 1:
    //                     fre_unserved -= x[i][node_set.match_amv[i][r]];
    //                     break;
    //             }
    //         }
    //     }
    // }
    // obj += w3 * (pas_unserved + fp * fre_unserved);

    mol.setObjective(obj, GRB_MINIMIZE);

    mol.update();
}


void Benchmark2::add_constraints(GRBModel &mol)
{
    //Constraint 1: Flow conservation that inflow is equal to the outflow of a time-space node
    for(int v = 0; v < amv_set.veh_num; v++)
    {
        for(int i = 1; i < node_set.nodenum; i++)
        {
            for(int t = 0; t < network.get_tsnodes()[i].size(); t++)
            {
                int ti = network.get_tsnodes()[i][t][1];
                vector<vector<int>> outflows_i_t = network.cap_outflow(i, ti);  //! {arc_type, arc_id}
                vector<vector<int>> inflows_i_t = network.cap_inflow(i, ti);
                GRBLinExpr outflow = 0.0, inflow = 0.0;
                for(int idx = 0; idx < outflows_i_t.size(); idx++)
                {
                    outflow += z[outflows_i_t[idx][0]][outflows_i_t[idx][1]][v];
                }
                for(int idx = 0; idx < inflows_i_t.size(); idx++)
                {
                    inflow += z[inflows_i_t[idx][0]][inflows_i_t[idx][1]][v];
                }
                mol.addConstr(outflow == inflow, "fc_(" + to_string(i) + "," + to_string(ti) + ")," + to_string(v));             
            }
        }
    }

    //Constraint 2: For each AMV, there is at most one moving arc flowing out of the central depot
    for(int v = 0; v < amv_set.veh_num; v++)
    {
        GRBLinExpr moveout = 0.0;
        for(int L = 0; L < out_depot_tsarcs.size(); L++)
        {
            moveout += z[0][out_depot_tsarcs[L]][v];
        }
        mol.addConstr(moveout <= 1, "om_" + to_string(v));
    }

    //Constraint 3: If an AMV is going to serve a customer of the same type, it will go through at most one corresponding serving arc of the customer
    //Constraint 4: If a customer and an AMV are of different type, the AMV will not serve this customer
    for(int i = 1; i < node_set.nodenum; i++)
    {
        if(node_set.nodetype[i] != 2) //if nodetype == 2 -> no serving arcs
        {
            GRBLinExpr serve_i_by_all_vehs;
            for(int v = 0; v < amv_set.veh_num; v++)
            {
                GRBLinExpr serve_flow = 0.0;
                vector<vector<vector<int>>> ser_arcs = network.cap_serarc_cus(i);
                vector<int> pos_arcser = network.intersect_pos(network.get_tsarcs()[2], ser_arcs);
                for(int L = 0; L < pos_arcser.size(); L++)
                {
                    serve_flow += z[2][pos_arcser[L]][v];
                }
                //contraint 3
                mol.addConstr(serve_flow == x[i][v], "sv_" + to_string(i) + "," + to_string(v));
                if(node_set.nodetype[i] != amv_set.veh_type[v])
                {
                    //constraint 4
                    mol.addConstr(x[i][v] == 0, "unmatch_" + to_string(i) + "," + to_string(v));
                }
                serve_i_by_all_vehs += x[i][v];
            }
            //constraint 3: at most serve by one vehicle
            mol.addConstr(serve_i_by_all_vehs == 1, "servemost_" + to_string(i));   //serve_i_by_all_vehs <= 1
        }
    }

    //Constraint 5 and 6: The relationship of moving arcs and waiting arcs with serving arcs
    for(int i = 1; i < node_set.nodenum; i++)
    {
        if(node_set.nodetype[i] != 2)
        {
            vector<int> serve_tidx = network.get_sernodes_id_in_tsnodes(i);
            for(int t = 0; t < network.get_tsnodes_ser()[i].size(); t++) //t < network.get_tsnodes_ser()[i].size() - node_set.service_time[i]
            {
                int time_i = network.get_tsnodes_ser()[i][t][1]; //! start serving time
                vector<vector<int>> serveout_i_t = network.cap_outflow(i,time_i,2);
                vector<vector<int>> movein_i_t = network.cap_inflow(i,time_i,0);
                for(int r = 0; r < node_set.match_amv[i].size(); r++)
                {
                    int v = node_set.match_amv[i][r];
                    GRBLinExpr serve_arc = 0.0, move_arc = 0.0, wait_arc = 0.0;
                    serve_arc = z[2][serveout_i_t[0][1]][v];
                    for(int idx = 0; idx < movein_i_t.size(); idx++)
                    {
                        move_arc += z[0][movein_i_t[idx][1]][v];
                    }
                    if((t > 0) || (serve_tidx[t] == 0))  //t > start serving time, or, time_ti == network.get_tsnodes()[i][0][1] (waiting arcs do not exist)
                    {
                        mol.addConstr(move_arc >= serve_arc, "m-s_(" + to_string(i) + "," + to_string(time_i) + ")" + "," + to_string(v));
                    }
                    else    //(t = start serving time) and (time_ti > network.get_tsnodes()[i][0][1])
                    {
                        vector<vector<int>> waitin_i_t = network.cap_inflow(i,time_i,1);
                        wait_arc = z[1][waitin_i_t[0][1]][v];
                        mol.addConstr(move_arc + wait_arc >= serve_arc, "mw-s_(" + to_string(i) + "," + to_string(time_i) + ")" + "," + to_string(v));
                    }
                }
            }
        }
    }

    //Constraint 7: The waiting time of an AMV of different types at each customer location should be within the type-specific time limit
    for(int v = 0; v < amv_set.veh_num; v++)
    {
        int wait_lim = (amv_set.veh_type[v] == 0) ? WAIT_0 : WAIT_1;
        for(int i = 1; i < node_set.nodenum; i++)
        {
            for(int t = wait_lim+1; t < network.get_tsnodes()[i].size(); t++)
            {
                GRBLinExpr wait_arcs = 0;
                int time_i = network.get_tsnodes()[i][t][1];
                vector<vector<int>> inarc_idx = network.cap_inflow(i,time_i,1); //find the index of the inflow waiting arc in the complete set of all waiting arcs
                for(int k = 0; k <= wait_lim; k++)
                {
                    wait_arcs += z[1][inarc_idx[0][1]-k][v];
                }
                mol.addConstr(wait_arcs <= wait_lim, "wlim_" + to_string(v) + "," + to_string(i) + "," + to_string(t));
            }
        }
    }

    //Constraint 8: The total waiting time of a (passenger-type) AMV at all customer locations should be within the time limit
    for(int v = 0; v < amv_set.veh_num; v++)
    {
        GRBLinExpr wait_flow = 0.0;
        for(int i = 1; i < node_set.nodenum; i++)
        {
            vector<vector<vector<int>>> wait_arcs = network.cap_waitarc_cus(i);
            vector<int> pos_arcwait = network.intersect_pos(network.get_tsarcs()[1], wait_arcs);
            for(int L = 0; L < pos_arcwait.size(); L++)
            {
                wait_flow += z[1][pos_arcwait[L]][v];
            }
        }
        int wait_max = (amv_set.veh_type[v] == 0) ? WAIT_MAX_0 : WAIT_MAX_1;
        mol.addConstr(wait_flow <= wait_max, "wmax_" + to_string(v));
    }

    //Constraint 9: The total distance traveled by each AMV should be within the range limit
    for(int v = 0; v < amv_set.veh_num; v++)  
    {    
        GRBLinExpr total_dist = 0.0;
        for(int L = 0; L < network.get_tsarcs()[0].size(); L++)
        {
            int s0 = network.get_tsarcs()[0][L][0][0];
            int s1 = network.get_tsarcs()[0][L][1][0];   
            total_dist += node_set.initial_distance[s0][s1] * z[0][L][v];
        }
        mol.addConstr(total_dist <= amv_set.max_range, "range_" + to_string(v));
    }

    //Constraint 10: AMV load limitation
    for(int a = 0; a < network.get_tsarcs().size(); a++)
    {
        for(int L = 0; L < network.get_tsarcs()[a].size(); L++)
        {
            for(int v = 0; v < amv_set.veh_num; v++)
            {
                mol.addConstr(E[a][L][v] <= amv_set.veh_cap[v] * z[a][L][v], 
                    "load_" + to_string(a) + "," + to_string(L) + "," + to_string(v));
            }
        }
    }

    //Constraint 11: initial vehicle load
    for(int v = 0; v < amv_set.veh_num; v++)
    {
        GRBLinExpr init_load = 0.0, serve_delivery = 0.0;
        for(int L = 0; L < out_depot_tsarcs.size(); L++)
        {
            init_load += E[0][out_depot_tsarcs[L]][v];
        }
        for(int i = 1; i < node_set.nodenum; i++)
        {
            if(node_set.nodetype[i] != 2 && node_set.demand_type[i] == 1) //demand_type = 1: delivery nodes
            {
                vector<int> serve_tidx = network.get_sernodes_id_in_tsnodes(i);
                for(int t = 0; t < network.get_tsnodes_ser()[i].size(); t++) //t < network.get_tsnodes_ser()[i].size() - node_set.service_time[i]
                {
                    int ti_in_nodes = serve_tidx[t];
                    serve_delivery -= e[i][ti_in_nodes][v];
                }
            }
        }
        mol.addConstr(init_load == serve_delivery, "initd_" + to_string(v));
    }

    //Constraint 12: service amount related with serving arcs and customer demands
    for(int i = 1; i < node_set.nodenum; i++)
    {
        if(node_set.nodetype[i] != 2)
        {
            vector<int> serve_tid = network.get_sernodes_id_in_tsnodes(i);
            int start_tid = serve_tid.front();   //the index of earliest serving start time in tsnodes -> node_set.service_tw[i][0]
            int end_tid = serve_tid.back();   //the index of lastest serving start time in tsnodes -> node_set.service_tw[i][1]
            for(int t = 0; t < network.get_tsnodes()[i].size(); t++)
            {
                int time_i = network.get_tsnodes()[i][t][1];
                for(int v = 0; v < amv_set.veh_num; v++)
                {
                    if(t < start_tid || t > end_tid)
                    {
                        mol.addConstr(e[i][t][v] == 0, "amountlim_" + to_string(i) + "," + to_string(time_i) + "," + to_string(v));
                    }
                    else
                    {
                        vector<vector<int>> serveout_i_t = network.cap_outflow(i,time_i,2);
                        {
                            GRBLinExpr serve_arc = 0.0;
                            serve_arc = z[2][serveout_i_t[0][1]][v];
                            mol.addConstr(e[i][t][v] == node_set.demands[i] * serve_arc, "amountlim_" + to_string(i) + "," + to_string(time_i) + "," + to_string(v));
                        }
                    }
                }
            }
        }
        else
        {
            for(int t = 0; t < network.get_tsnodes()[i].size(); t++)
            {
                int time_i = network.get_tsnodes()[i][t][1];
                for(int v = 0; v < amv_set.veh_num; v++)
                {
                    mol.addConstr(e[i][t][v] == 0, "amountlim_" + to_string(i) + "," + to_string(time_i) + "," + to_string(v));
                }
            }
        }
    }

    //Constraint 13: Flow balance of AMV load
    for(int v = 0; v < amv_set.veh_num; v++)
    {
        for(int i = 1; i < node_set.nodenum; i++)
        {
            for(int t = 0; t < network.get_tsnodes()[i].size(); t++)
            {
                GRBLinExpr load_outflow = 0.0, load_inflow = 0.0;
                int time_ti = network.get_tsnodes()[i][t][1];
                vector<vector<int>> outarc_idx = network.cap_outflow(i,time_ti);
                vector<vector<int>> inarc_idx = network.cap_inflow(i,time_ti);
                for(int idx = 0; idx < outarc_idx.size(); idx++)
                {
                    load_outflow += E[outarc_idx[idx][0]][outarc_idx[idx][1]][v];
                }
                for(int idx = 0; idx < inarc_idx.size(); idx++)
                {
                    load_inflow += E[inarc_idx[idx][0]][inarc_idx[idx][1]][v];
                }
                mol.addConstr(load_outflow - load_inflow == e[i][t][v], "dflow_" + to_string(i) + "," + to_string(time_ti) + "," + to_string(v));
            }
        }
    }

    mol.update();
}

void Benchmark2::get_result(GRBModel &mol)
{
    //write the result to a dat file
    optimstatus = mol.get(GRB_IntAttr_Status);
    outfile.open(result_file, ios::out | ios::trunc);

    outfile << "Optimization complete" << endl;

    //output the optimization result
    duration = (clock() - start ) / (double) CLOCKS_PER_SEC;
    outfile << "Optimization took "<< double(duration) << " seconds." << endl;

    if (optimstatus == GRB_OPTIMAL) 
    {
        //report objective values
        outfile << "Optimal objective: " << mol.get(GRB_DoubleAttr_ObjVal) << endl;

        //report total distance
        obj_dist = 0.0;
        for(int L = 0; L < network.get_tsarcs()[0].size(); L++)  
        {  
            int s0 = network.get_tsarcs()[0][L][0][0];
            int s1 = network.get_tsarcs()[0][L][1][0];
            for(int v = 0; v < amv_set.veh_num; v++)  
            {    
                obj_dist += node_set.initial_distance[s0][s1] * z[0][L][v].get(GRB_DoubleAttr_X); 
            }
        }
        outfile << "  Total distance: " << obj_dist << endl;

        //report total unserved requests of passenger and freight customers
        unsv_pas = 0.0, unsv_fre = 0.0;
        for(int i = 1; i < node_set.nodenum; i++)
        {
            if(node_set.nodetype[i] == 0) unsv_pas += 1;
            else if (node_set.nodetype[i] == 1) unsv_fre += 1;
        }
        for(int i = 1; i < node_set.nodenum; i++)   //all time-space service nodes
        {  
            if(node_set.nodetype[i] != 2)
            {
                for(int r = 0; r < node_set.match_amv[i].size(); r++)    //node_set.match_amv[i]
                {
                    switch(node_set.nodetype[i])
                    {
                        case 0:
                            unsv_pas -= x[i][node_set.match_amv[i][r]].get(GRB_DoubleAttr_X);
                            break;
                        case 1:
                            unsv_fre -= x[i][node_set.match_amv[i][r]].get(GRB_DoubleAttr_X);
                            break;
                    }
                }
            }
        }

        outfile << "  Total passenger unserved requests: " << unsv_pas << endl;
        outfile << "  Total freight unserved requests: " << unsv_fre << endl;

        //report the vehicles' and passengers' total trip duration
        mavs_trip_duration = 0.0;
        for(int v = 0; v < amv_set.veh_num; v++)
        {
            double in_time = 0.0, out_time = 0.0; 
            for(int L = 0; L < in_depot_tsarcs.size(); L++)
            {
                int t1 = network.get_tsarcs()[0][in_depot_tsarcs[L]][1][1];
                in_time += z[0][in_depot_tsarcs[L]][v].get(GRB_DoubleAttr_X) * t1;
            }
            for(int L = 0; L < out_depot_tsarcs.size(); L++)
            {
                int t0 = network.get_tsarcs()[0][out_depot_tsarcs[L]][0][1];
                out_time += z[0][out_depot_tsarcs[L]][v].get(GRB_DoubleAttr_X) * t0;
            }
            mavs_trip_duration += (in_time - out_time);
        }
        outfile << "  Vehicles' total trip duration: " <<  mavs_trip_duration << endl;

        //report the variable values
        outfile << "print out the value of z and y:" << endl;
        for(int a = 0; a < network.get_tsarcs().size(); a++)
        {
            for(int L = 0; L < network.get_tsarcs()[a].size(); L++)
            {
                for(int v = 0; v < amv_set.veh_num; v++)
                {
                    if(z[a][L][v].get(GRB_DoubleAttr_X) > 0.5)  //>=1
                    {
                        outfile << z[a][L][v].get(GRB_StringAttr_VarName) << " = "  << z[a][L][v].get(GRB_DoubleAttr_X) << endl;              
                    }
                }
            }
        }

        outfile << "print out the value of x:" << endl;
        for(int i = 1; i < node_set.nodenum; i++)
        {
            for(int v = 0; v < amv_set.veh_num; v++)  
            {  
                if(x[i][v].get(GRB_DoubleAttr_X) > 0.5)  //>=1
                {
                    outfile << x[i][v].get(GRB_StringAttr_VarName) << " = "  << x[i][v].get(GRB_DoubleAttr_X) << endl; 
                }
            }
        }

        outfile << "print out the value of E:" << endl;
        for(int a = 0; a < network.get_tsarcs().size(); a++)
        {
            for(int L = 0; L < network.get_tsarcs()[a].size(); L++)
            {
                for(int v = 0; v < amv_set.veh_num; v++)
                {
                    if(z[a][L][v].get(GRB_DoubleAttr_X) > 0.5)  //>=1
                    {
                        outfile << E[a][L][v].get(GRB_StringAttr_VarName) << " = "  << E[a][L][v].get(GRB_DoubleAttr_X) << endl; 
                    }
                }
            }
        }

        outfile << "print out the value of e:" << endl;
        for(int i = 1; i < node_set.nodenum; i++)
        {
            for(int t = 0; t < network.get_tsnodes()[i].size(); t++)
            {
                for(int v = 0; v < amv_set.veh_num; v++)
                {
                    if(e[i][t][v].get(GRB_DoubleAttr_X) > 0.5)  //>=1
                    {
                        outfile << e[i][t][v].get(GRB_StringAttr_VarName) << " = "  << e[i][t][v].get(GRB_DoubleAttr_X) << endl;
                    }
                }
            }
        }
    } 
    else if (optimstatus == GRB_INF_OR_UNBD) 
    {
        outfile << "Model is infeasible or unbounded" << endl;
    } 
    else if (optimstatus == GRB_INFEASIBLE) 
    {
        outfile << "Model is infeasible" << endl;
    } 
    else if (optimstatus == GRB_UNBOUNDED) 
    {
        outfile << "Model is unbounded" << endl;
    } 
    else 
    {
        outfile << "Optimization was stopped with status = " << optimstatus << endl;
    }
}