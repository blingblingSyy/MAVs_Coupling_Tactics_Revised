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
#include "benchmark1_1.h"
#include "initialize.h"
#include "utility.h"
#include "config.h"
using namespace std;  

//build the gurobi environment
Benchmark1_1::Benchmark1_1(Nodes nodes, Vehicles amvs, string modelname, TimeSpace_Network ts_network):Model(nodes, amvs, modelname, ts_network)
{
    int pas_mav_num = static_cast<int>(count(amv_set.veh_type.begin(), amv_set.veh_type.end(), 0));
    int fre_mav_num = static_cast<int>(count(amv_set.veh_type.begin(), amv_set.veh_type.end(), 1));
    mav_num_type = {pas_mav_num, fre_mav_num};
}

void Benchmark1_1::add_variables(GRBModel &mol)
{
    //x_{i,p}: let number of platoons be equal to the number of amvs
    x.resize(node_set.nodenum);
    for(int i = 1; i < node_set.nodenum; i++)   //add constraint: the depot and intersections will not be served
    {  
        x[i].resize(amv_set.veh_num);
        for(int p = 0; p < amv_set.veh_num; p++)
        {
            x[i][p] = mol.addVar(0.0, 1.0, 0, GRB_BINARY, "x" + to_string(i) + "," + to_string(p));
        }
    }

    //z_{l,p}, E_{l,p,k}, f_{l,p,k,u}, F_{l,p,h}
    auto arc_type = network.get_tsarcs().size();
    z.resize(arc_type);
    E.resize(arc_type);
    F.resize(arc_type);
    for(int a = 0; a < arc_type; a++)
    {
        auto arc_size = network.get_tsarcs()[a].size();
        z[a].resize(arc_size);
        E[a].resize(arc_size);
        F[a].resize(arc_size);
        for(int L = 0; L < arc_size; L++)
        {
            int s0 = network.get_tsarcs()[a][L][0][0];
            int t0 = network.get_tsarcs()[a][L][0][1];
            int s1 = network.get_tsarcs()[a][L][1][0];
            int t1 = network.get_tsarcs()[a][L][1][1];
            z[a][L].resize(amv_set.veh_num);
            E[a][L].resize(amv_set.veh_num);
            F[a][L].resize(amv_set.veh_num);
            for(int p = 0; p < amv_set.veh_num; p++)
            {
                z[a][L][p] = mol.addVar(0.0, 1.0, 0, GRB_BINARY, 
                        "z" + to_string(a) + ",(" + to_string(s0) + "," + to_string(t0) + "," + to_string(s1) + "," + to_string(t1)+ ")," + to_string(p));
                E[a][L][p].resize(VTYPE);
                F[a][L][p].resize(amv_set.max_plen+1);
                for(int k = 0; k < VTYPE; k++)
                {
                    E[a][L][p][k] = mol.addVar(0.0, INFINITY, 0, GRB_CONTINUOUS, 
                            "E" + to_string(a) + ",(" + to_string(s0) + "," + to_string(t0) + "," + to_string(s1) + "," + to_string(t1)+ ")," + to_string(p) + "," + to_string(k));
                }
                for(int h = 1; h <= amv_set.max_plen; h++)
                {
                    F[a][L][p][h] = mol.addVar(0.0, 1.0, 0, GRB_BINARY, 
                            "F" + to_string(a) + ",(" + to_string(s0) + "," + to_string(t0) + "," + to_string(s1) + "," + to_string(t1)+ ")," + to_string(p) + "," + to_string(h));
                }
            }
        }
    }

    //e_{(i,t),p}
    e.resize(node_set.nodenum);
    for(int i = 1; i < node_set.nodenum; i++)
    {
        e[i].resize(network.get_tsnodes()[i].size());
        for(int t = 0; t < network.get_tsnodes()[i].size(); t++)    //t = actual time - travel_tw[i][0]
        {
            int time_i = network.get_tsnodes()[i][t][1];
            e[i][t].resize(amv_set.veh_num);
            for(int p = 0; p < amv_set.veh_num; p++)
            {
                e[i][t][p] = mol.addVar(0.0, GRB_INFINITY, 0, GRB_INTEGER, "e(" + to_string(i) + "," + to_string(time_i) + ")," + to_string(p));
            }
        }
    }

    //y_{p,k,m}
    y.resize(amv_set.veh_num);
    for(int p = 0; p < amv_set.veh_num; p++)
    {
        y[p].resize(VTYPE);
        for(int k = 0; k < VTYPE; k++)
        {
            y[p][k].resize(mav_num_type[k]+1);
            for(int m = 1; m <= mav_num_type[k]; m++)
            {
                y[p][k][m] = mol.addVar(0.0, 1.0, 0, GRB_BINARY, "y" + to_string(p) + "," + to_string(k) + "," + to_string(m));
            }
        }
    }

    //g_{p,h}
    g.resize(amv_set.veh_num);
    for(int p = 0; p < amv_set.veh_num; p++)
    {
        g[p].resize(amv_set.max_plen+1);
        for(int h = 1; h <= amv_set.max_plen; h++)
        {
            g[p][h] = mol.addVar(0.0, 1.0, 0, GRB_BINARY, "g" + to_string(p) + "," + to_string(h));
        }
    }

    mol.update();
}

void Benchmark1_1::add_objectives(GRBModel &mol)
{
    //add objective to the model
    GRBQuadExpr obj = 0.0;
    
    //1st term: total distance
    for(int L = 0; L < network.get_tsarcs()[0].size(); L++)  
    {  
        int s0 = network.get_tsarcs()[0][L][0][0];
        int s1 = network.get_tsarcs()[0][L][1][0];
        for(int p = 0; p < amv_set.veh_num; p++)
        {
            for(int h = 1; h <= amv_set.max_plen; h++)
            {
                obj += w1 * node_set.initial_distance[s0][s1] * pl_factor(h) * F[0][L][p][h] * h;
            }
        }
    }

    // 2nd term: total trip duration
    for(int p = 0; p < amv_set.veh_num; p++)
    {
        GRBLinExpr in_time = 0.0, out_time = 0.0;
        for(int L = 0; L < in_depot_tsarcs.size(); L++)
        {
            int t1 = network.get_tsarcs()[0][in_depot_tsarcs[L]][1][1];
            for(int h = 1; h <= amv_set.max_plen; h++)
            {
                in_time += F[0][in_depot_tsarcs[L]][p][h] * h * t1;
            }       
        }
        for(int L = 0; L < out_depot_tsarcs.size(); L++)
        {
            int t0 = network.get_tsarcs()[0][out_depot_tsarcs[L]][0][1];    
            for(int h = 1; h <= amv_set.max_plen; h++)
            {
                out_time += F[0][out_depot_tsarcs[L]][p][h] * h * t0;
            }
        }
        obj += w2 * (in_time - out_time);
    }

    //3rd term: unserved demands
    GRBLinExpr pas_unserved = 0.0, fre_unserved = 0.0;
    for(int i = 1; i < node_set.nodenum; i++)
    {
        if(node_set.nodetype[i] == 0) pas_unserved += 1;
        else if (node_set.nodetype[i] == 1) fre_unserved += 1;
    }
    for(int i = 1; i < node_set.nodenum; i++)   //all time-space service nodes
    {  
        if(node_set.nodetype[i] != 2)
        {
            for(int p = 0; p < amv_set.veh_num; p++)    //node_set.match_amv[i]
            {
                switch(node_set.nodetype[i])
                {
                    case 0:
                        pas_unserved -= x[i][p];
                        break;
                    case 1:
                        fre_unserved -= x[i][p];
                        break;
                }
            }
        }
    }
    obj += w3 * (pas_unserved + fp * fre_unserved);

    mol.setObjective(obj, GRB_MINIMIZE);

    mol.update();
}

void Benchmark1_1::add_constraints(GRBModel &mol)
{
    //Constraint 1: Flow conservation that inflow is equal to the outflow of a time-space node for each platoon
    for(int p = 0; p < amv_set.veh_num; p++)
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
                    outflow += z[outflows_i_t[idx][0]][outflows_i_t[idx][1]][p];
                }
                for(int idx = 0; idx < inflows_i_t.size(); idx++)
                {
                    inflow += z[inflows_i_t[idx][0]][inflows_i_t[idx][1]][p];
                }
                mol.addConstr(outflow == inflow, "fc_(" + to_string(i) + "," + to_string(ti) + ")," + to_string(p));             
            }
        }
    }

    //Constraint 2: For each platoon, there is at most one moving arc flowing out of the central depot
    for(int p = 0; p < amv_set.veh_num; p++)
    {
        GRBLinExpr moveout = 0.0;
        for(int L = 0; L < out_depot_tsarcs.size(); L++)
        {
            moveout += z[0][out_depot_tsarcs[L]][p];
        }
        mol.addConstr(moveout <= 1, "om_" + to_string(p));
    }

    //Constraint 3: If a platoon is going to serve a customer, it will go through one corresponding serving arc of the customer
    //Constraint 4: a customer will be served by at most one platoon
    for(int i = 1; i < node_set.nodenum; i++)
    {
        if(node_set.nodetype[i] != 2) //if nodetype == 2 -> no serving arcs
        {
            GRBLinExpr serve_i_by_all_platoons;
            for(int p = 0; p < amv_set.veh_num; p++)
            {
                GRBLinExpr serve_flow = 0.0;
                vector<vector<vector<int>>> ser_arcs = network.cap_serarc_cus(i);
                vector<int> pos_arcser = network.intersect_pos(network.get_tsarcs()[2], ser_arcs);
                for(int L = 0; L < pos_arcser.size(); L++)
                {
                    serve_flow += z[2][pos_arcser[L]][p];
                }
                //contraint 3
                mol.addConstr(serve_flow == x[i][p], "sv_" + to_string(i) + "," + to_string(p));
                serve_i_by_all_platoons += x[i][p];
            }
            //constraint 3: at most serve by one vehicle
            mol.addConstr(serve_i_by_all_platoons == 1, "servemost_" + to_string(i));  //serve_i_by_all_platoons <= 1
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
                for(int p = 0; p < node_set.match_amv[i].size(); p++)
                {
                    GRBLinExpr serve_arc = 0.0, move_arc = 0.0, wait_arc = 0.0;
                    serve_arc = z[2][serveout_i_t[0][1]][p];
                    for(int idx = 0; idx < movein_i_t.size(); idx++)
                    {
                        move_arc += z[0][movein_i_t[idx][1]][p];
                    }
                    if((t > 0) || (serve_tidx[t] == 0))  //t > start serving time, or, time_ti == network.get_tsnodes()[i][0][1] (waiting arcs do not exist)
                    {
                        mol.addConstr(move_arc >= serve_arc, "m-s_(" + to_string(i) + "," + to_string(time_i) + ")" + "," + to_string(p));
                    }
                    else    //(t = start serving time) and (time_ti > network.get_tsnodes()[i][0][1])
                    {
                        vector<vector<int>> waitin_i_t = network.cap_inflow(i,time_i,1);
                        wait_arc = z[1][waitin_i_t[0][1]][p];
                        mol.addConstr(move_arc + wait_arc >= serve_arc, "mw-s_(" + to_string(i) + "," + to_string(time_i) + ")" + "," + to_string(p));
                    }
                }
            }
        }
    }

    // Constraint 7: The waiting time of an AMV of different types at each customer location should be within the type-specific time limit
    // Constraint 8: The total waiting time of a (passenger-type) AMV at all customer locations should be within the time limit
    for(int p = 0; p < amv_set.veh_num; p++)
    {
        for(int k = 0; k < VTYPE; k++)
        {
            GRBLinExpr wait_lim = 1;
            for(int m = 1; m <= mav_num_type[k]; m++)
            {
                wait_lim -= y[p][k][m];
            }
            int wait_max = (k == 0) ? WAIT_MAX_0 : WAIT_MAX_1;
            wait_lim *= wait_max;
            int wait_pernode = (k == 0) ? WAIT_0 : WAIT_1;
            GRBLinExpr wait_lim_per_node = wait_lim + wait_pernode;
            GRBLinExpr wait_lim_max = wait_lim + wait_max;
            
            for(int i = 1; i < node_set.nodenum; i++)
            {
                for(int t = wait_pernode+1; t < network.get_tsnodes()[i].size(); t++)
                {
                    GRBLinExpr wait_arcs = 0;
                    int time_i = network.get_tsnodes()[i][t][1];
                    vector<vector<int>> inarc_idx = network.cap_inflow(i,time_i,1); //find the index of the inflow waiting arc in the complete set of all waiting arcs
                    for(int w = 0; w <= wait_pernode; w++)
                    {
                        wait_arcs += z[1][inarc_idx[0][1]-w][p];
                    }
                    // constraint 7
                    mol.addConstr(wait_arcs <= wait_lim_per_node, "wlim_" + to_string(p) + "," + to_string(i) + "," + to_string(t));
                }
            }

            GRBLinExpr all_wait_flow = 0.0;
            for(int i = 1; i < node_set.nodenum; i++)
            {
                vector<vector<vector<int>>> wait_arcs = network.cap_waitarc_cus(i);
                vector<int> pos_arcwait = network.intersect_pos(network.get_tsarcs()[1], wait_arcs);
                for(int L = 0; L < pos_arcwait.size(); L++)
                {
                    all_wait_flow += z[1][pos_arcwait[L]][p];
                }
            }
            // constraint 8
            mol.addConstr(all_wait_flow <= wait_lim_max, "wmax_" + to_string(p) + ", " + to_string(k));
        }
    }

    // //Constraint 9: The total distance traveled by each AMV should be within the range limit
    // for(int p = 0; p < amv_set.veh_num; p++)  
    // {    
    //     GRBLinExpr total_dist = 0.0;
    //     for(int L = 0; L < network.get_tsarcs()[0].size(); L++)
    //     {
    //         int s0 = network.get_tsarcs()[0][L][0][0];
    //         int s1 = network.get_tsarcs()[0][L][1][0];   
    //         total_dist += node_set.initial_distance[s0][s1] * z[0][L][p];
    //     }
    //     mol.addConstr(total_dist <= amv_set.max_range, "range_" + to_string(p));
    // }

    //Constraint 10: only one platoon will be formed on each arc
    for(int a = 0; a < network.get_tsarcs().size(); a++)
    {
        for(int L = 0; L < network.get_tsarcs()[a].size(); L++)
        {
            GRBLinExpr plnum = 0.0;
            for(int p = 0; p < amv_set.veh_num; p++)
            {
                plnum += z[a][L][p];
            }
            mol.addConstr(plnum <= 1, "plnum_" + to_string(a) + "," + to_string(L));
        }
    }

    //Constraint 11: if a platoon comes out from depot, it must have a specific length
    for(int p = 0; p < amv_set.veh_num; p++)
    {
        GRBLinExpr moveout = 0.0;
        for(int L = 0; L < out_depot_tsarcs.size(); L++)
        {
            moveout += z[0][out_depot_tsarcs[L]][p];
        }
        GRBLinExpr platoon_len = 0.0;
        for(int h = 1; h <= amv_set.max_plen; h++)
        {
            platoon_len += g[p][h];
        }
        mol.addConstr(moveout == platoon_len, "om_" + to_string(p));
    }

    //Constraint 12: there's at most one specific number of MAVs of each type for a platoon
    for(int p = 0; p < amv_set.veh_num; p++)
    {
        for(int k = 0; k < VTYPE; k++)
        {
            GRBLinExpr vnum = 0.0;
            for(int m = 1; m <= mav_num_type[k]; m++)  
            {  
                vnum += y[p][k][m];
            }
            mol.addConstr(vnum <= 1, "vehnumk_" + to_string(p) + "," + to_string(k));       
        }
    }

    //Constraint 13: balance of the platoon length and the total number of mavs included
    for(int p = 0; p < amv_set.veh_num; p++)
    {
        GRBLinExpr total_vnum = 0.0, pl_len = 0.0;
        for(int k = 0; k < VTYPE; k++)
        {
            for(int m = 1; m <= mav_num_type[k]; m++)  
            {  
                total_vnum += y[p][k][m] * m;
            }
        }
        for(int h = 1; h <= amv_set.max_plen; h++)
        {
            pl_len += g[p][h] * h;
        }
        mol.addConstr(total_vnum == pl_len, "vehpl_" + to_string(p));       
    }

    //Constraint 14: there's at most one specific length for a platoon
    for(int p = 0; p < amv_set.veh_num; p++)
    {
        GRBLinExpr pl_len = 0.0;
        for(int h = 1; h <= amv_set.max_plen; h++)
        {
            pl_len += g[p][h];
        }
        mol.addConstr(pl_len <= 1, "plen_" + to_string(p));       
    }

    //Constraint 15: the number of mavs used in all platoons should be within the limit of specific type
    for(int k = 0; k < VTYPE; k++)
    {
        GRBLinExpr total_veh_k;
        for(int p = 0; p < amv_set.veh_num; p++)
        {
            for(int m = 1; m <= mav_num_type[k]; m++)
            {
                total_veh_k += y[p][k][m] * m;
            }
        }
        mol.addConstr(total_veh_k <= mav_num_type[k], "vehlimk_" + to_string(k));
    }

    //Constraint 16-18: relationship between z, g, and F
    for(int a = 0; a < network.get_tsarcs().size(); a++)
    {
        for(int L = 0; L < network.get_tsarcs()[a].size(); L++)
        {
            for(int p = 0; p < amv_set.veh_num; p++)
            {
                for(int h = 1; h <= amv_set.max_plen; h++)
                {
                    mol.addConstr(F[a][L][p][h] <= z[a][L][p], "Fz_" + 
                                    to_string(a) + "," + to_string(L) + "," + to_string(p) + "," + to_string(h));
                    mol.addConstr(F[a][L][p][h] <= g[p][h], "Fg_" + 
                                    to_string(a) + "," + to_string(L) + "," + to_string(p) + "," + to_string(h));
                    mol.addConstr(F[a][L][p][h] >= g[p][h] + z[a][L][p] - 1, "Fgz_" + 
                                    to_string(a) + "," + to_string(L) + "," + to_string(p) + "," + to_string(h));
                }
            }
        }
    }

    //Constraint 19: AMV load limitation
    int veh_cap_max = VCAP_1 * amv_set.veh_num;
    for(int a = 0; a < network.get_tsarcs().size(); a++)
    {
        for(int L = 0; L < network.get_tsarcs()[a].size(); L++)
        {
            for(int p = 0; p < amv_set.veh_num; p++)
            {
                for(int k = 0; k < VTYPE; k++)
                {
                    int veh_cap = (k == 0) ? VCAP_0 : VCAP_1;
                    GRBLinExpr total_veh_k;
                    for(int m = 1; m <= mav_num_type[k]; m++)
                    {
                        total_veh_k += y[p][k][m] * m;
                    }
                    mol.addConstr(E[a][L][p][k] <= veh_cap * total_veh_k + (1 - z[a][L][p]), 
                                    "load1_" + to_string(a) + "," + to_string(L) + "," + to_string(p) + "," + to_string(k));
                    mol.addConstr(E[a][L][p][k] <= veh_cap_max * z[a][L][p], 
                                    "load2_" + to_string(a) + "," + to_string(L) + "," + to_string(p) + "," + to_string(k));
                }
            }
        }
    }

    //Constraint 20: initial vehicle load
    for(int p = 0; p < amv_set.veh_num; p++)
    {
        for(int k = 0; k < VTYPE; k++)
        {
            GRBLinExpr init_load = 0.0, serve_delivery = 0.0;
            for(int L = 0; L < out_depot_tsarcs.size(); L++)
            {
                init_load += E[0][out_depot_tsarcs[L]][p][k];
            }
            for(int i = 1; i < node_set.nodenum; i++)
            {
                if(node_set.nodetype[i] == k && node_set.demand_type[i] == 1) //demand_type = 1: delivery nodes
                {
                    vector<int> serve_tidx = network.get_sernodes_id_in_tsnodes(i);
                    for(int t = 0; t < network.get_tsnodes_ser()[i].size(); t++) //t < network.get_tsnodes_ser()[i].size() - node_set.service_time[i]
                    {
                        int ti_in_nodes = serve_tidx[t];
                        serve_delivery -= e[i][ti_in_nodes][p];
                    }
                }
            }
            mol.addConstr(init_load == serve_delivery, "initlk_" + to_string(p) + "," + to_string(k));
        }
    }

    //Constraint 21: service amount related with serving arcs and customer demands
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
                for(int p = 0; p < amv_set.veh_num; p++)
                {
                    if(t < start_tid || t > end_tid)
                    {
                        mol.addConstr(e[i][t][p] == 0, "amountlim_" + to_string(i) + "," + to_string(time_i) + "," + to_string(p));
                    }
                    else
                    {
                        vector<vector<int>> serveout_i_t = network.cap_outflow(i,time_i,2);
                        {
                            GRBLinExpr serve_arc = 0.0;
                            serve_arc = z[2][serveout_i_t[0][1]][p];
                            mol.addConstr(e[i][t][p] == node_set.demands[i] * serve_arc, "amountlim_" + to_string(i) + "," + to_string(time_i) + "," + to_string(p));
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
                for(int p = 0; p < amv_set.veh_num; p++)
                {
                    mol.addConstr(e[i][t][p] == 0, "amountlim_" + to_string(i) + "," + to_string(time_i) + "," + to_string(p));
                }
            }
        }
    }

    //Constraint 22: Flow balance of AMV load
    for(int p = 0; p < amv_set.veh_num; p++)
    {
        for(int k = 0; k < VTYPE; k++)
        {
            for(int i = 1; i < node_set.nodenum; i++)
            {
                for(int t = 0; t < network.get_tsnodes()[i].size(); t++)
                {
                    int time_ti = network.get_tsnodes()[i][t][1];
                    GRBLinExpr load_outflow = 0.0, load_inflow = 0.0;
                    vector<vector<int>> outarc_idx = network.cap_outflow(i,time_ti);
                    vector<vector<int>> inarc_idx = network.cap_inflow(i,time_ti);
                    for(int idx = 0; idx < outarc_idx.size(); idx++)
                    {
                        load_outflow += E[outarc_idx[idx][0]][outarc_idx[idx][1]][p][k];
                    }
                    for(int idx = 0; idx < inarc_idx.size(); idx++)
                    {
                        load_inflow += E[inarc_idx[idx][0]][inarc_idx[idx][1]][p][k];
                    }
                    if(node_set.nodetype[i] == k)
                    {
                        mol.addConstr(load_outflow - load_inflow == e[i][t][p], "dflow_" + to_string(i) + "," + to_string(time_ti) + "," + to_string(p));
                    }
                    else
                    {
                        mol.addConstr(load_outflow - load_inflow == 0, "dflow_" + to_string(i) + "," + to_string(time_ti) + "," + to_string(p));
                    }
                }
            }
        }
    }

    mol.update();

}

void Benchmark1_1::get_result(GRBModel &mol)
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
            for(int p = 0; p < amv_set.veh_num; p++)
            {
                for(int h = 1; h <= amv_set.max_plen; h++)
                {
                    obj_dist += node_set.initial_distance[s0][s1] * pl_factor(h) * F[0][L][p][h].get(GRB_DoubleAttr_X) * h; 
                }
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
                for(int p = 0; p < amv_set.veh_num; p++)    //node_set.match_amv[i]
                {
                    switch(node_set.nodetype[i])
                    {
                        case 0:
                            unsv_pas -= x[i][p].get(GRB_DoubleAttr_X);
                            break;
                        case 1:
                            unsv_fre -= x[i][p].get(GRB_DoubleAttr_X);
                            break;
                    }
                }
            }
        }

        outfile << "  Total passenger unserved requests: " << unsv_pas << endl;
        outfile << "  Total freight unserved requests: " << unsv_fre << endl;

        //report the vehicles' total trip duration
        mavs_trip_duration = 0.0;
        for(int p = 0; p < amv_set.veh_num; p++)
        {
            double in_time = 0.0, out_time = 0.0;
            for(int L = 0; L < in_depot_tsarcs.size(); L++)
            {
                int t1 = network.get_tsarcs()[0][in_depot_tsarcs[L]][1][1];     
                for(int h = 1; h <= amv_set.max_plen; h++)
                {
                    in_time += F[0][in_depot_tsarcs[L]][p][h].get(GRB_DoubleAttr_X) * h * t1;
                }       
            }
            for(int L = 0; L < out_depot_tsarcs.size(); L++)
            {
                int t0 = network.get_tsarcs()[0][out_depot_tsarcs[L]][0][1];    
                for(int h = 1; h <= amv_set.max_plen; h++)
                {
                    out_time += F[0][out_depot_tsarcs[L]][p][h].get(GRB_DoubleAttr_X) * h * t0;
                }            
            }
            mavs_trip_duration += in_time - out_time;
        }

        outfile << "  Vehicles' total trip duration: " <<  mavs_trip_duration << endl;

        //report the variable values
        outfile << "print out the value of z and F:" << endl;
        for(int a = 0; a < network.get_tsarcs().size(); a++)
        {
            for(int L = 0; L < network.get_tsarcs()[a].size(); L++)
            {
                for(int p = 0; p < amv_set.veh_num; p++)
                {
                    if(z[a][L][p].get(GRB_DoubleAttr_X) > 0.5)  //>=1
                    {
                        outfile << z[a][L][p].get(GRB_StringAttr_VarName) << " = "  << z[a][L][p].get(GRB_DoubleAttr_X) << endl;
                    }
                    for(int h = 1; h <= amv_set.max_plen; h++)
                    {
                        if(F[a][L][p][h].get(GRB_DoubleAttr_X) > 0.5)
                        {
                            outfile << F[a][L][p][h].get(GRB_StringAttr_VarName) << " = "  << F[a][L][p][h].get(GRB_DoubleAttr_X) << endl;
                        } 
                    }
                }
            }
        }
        
        outfile << "print out the value of g and y: " << endl;
        for(int p = 0; p < amv_set.veh_num; p++)
        {
            for(int h = 1; h <= amv_set.max_plen; h++)
            {
                if(g[p][h].get(GRB_DoubleAttr_X) > 0.5)
                {
                    outfile << g[p][h].get(GRB_StringAttr_VarName) << " = "  << g[p][h].get(GRB_DoubleAttr_X) << endl;
                    for(int k = 0; k < VTYPE; k++)
                    {
                        for(int m = 1; m <= mav_num_type[k]; m++)
                        {
                            if(y[p][k][m].get(GRB_DoubleAttr_X) > 0.5)
                            {
                                outfile << y[p][k][m].get(GRB_StringAttr_VarName) << " = "  << y[p][k][m].get(GRB_DoubleAttr_X) << endl;
                            }
                        }   
                    }
                }
            }
        }

        outfile << "print out the value of x:" << endl;
        for(int i = 1; i < node_set.nodenum; i++)
        {
            for(int p = 0; p < amv_set.veh_num; p++)  
            {  
                if(x[i][p].get(GRB_DoubleAttr_X) > 0.5)  //>=1
                {
                    outfile << x[i][p].get(GRB_StringAttr_VarName) << " = "  << x[i][p].get(GRB_DoubleAttr_X) << endl; 
                }
            }
        }

        outfile << "print out the value of E:" << endl;
        for(int a = 0; a < network.get_tsarcs().size(); a++)
        {
            for(int L = 0; L < network.get_tsarcs()[a].size(); L++)
            {
                for(int p = 0; p < amv_set.veh_num; p++)
                {
                    if(z[a][L][p].get(GRB_DoubleAttr_X) > 0.5)  //>=1
                    {
                        for(int k = 0; k < VTYPE; k++)
                        {
                            outfile << E[a][L][p][k].get(GRB_StringAttr_VarName) << " = "  << E[a][L][p][k].get(GRB_DoubleAttr_X) << endl; 
                        }
                    }
                }
            }
        }

        outfile << "print out the value of e:" << endl;
        for(int i = 1; i < node_set.nodenum; i++)
        {
            for(int t = 0; t < network.get_tsnodes()[i].size(); t++)
            {
                for(int p = 0; p < amv_set.veh_num; p++)
                {
                    if(e[i][t][p].get(GRB_DoubleAttr_X) > 0.5)  //>=1
                    {
                        outfile << e[i][t][p].get(GRB_StringAttr_VarName) << " = "  << e[i][t][p].get(GRB_DoubleAttr_X) << endl;
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