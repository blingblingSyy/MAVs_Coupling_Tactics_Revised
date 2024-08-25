# Project Title

Coupling and Decoupling Tactics of Autonomous Modular Vehicles for Integrated Passenger and Freight Transport


## Project Description

* The code designs coupling and decoupling tactics of Autonomous Modular Vehicles (AMVs) for integrated passenger and freight transport based on a transport system with pre-defined corridors. When multiple AMVs of different types travel along a path at the same time, they automatically form a platoon. 

* The model is a Mixed Integer Non-linear Programming model based on a multi-commodity time-space network, which can capture the waiting time of an AMV at a node to couple with other AMVs.

* The result of optimization will be output to the "result.dat" file.

### Dataset

* Network information
    * a simulated road network with pre-defined corridors and the distance of each corridor. Apart from the pre-defined central depot and customer locations, intersections are introduced in the system. 

* Customer information
    * Customer demands
    * Customer service time
    * Customer service time window

* AMV information
    * AMV speed
    * AMV maximum range
    * AMV numbers
    * AMV types: assign an AMV of passenger-type or freight-type with 0.5:0.5 likelihood
    * Maximum AMVs involved in a platoon
    * AMV capacity: type-specific
    * AMV waiting time limit per node & maximum waiting time limit for the entire route for each AMV

### corridor.txt
* 1st line: total number of amvs and maximum platoon length
* 2nd line: planning horizon;
* 3rd line: total number of nodes;
* 4th line: node type (0: passenger; 1: freight; 2: non-type)
* 5th line: demand type (0: pickup; 1: delivery)
* others: initial distance matrix (-1 means that two nodes are not connected)

### Time-space network
* Time-space nodes: in the form of $\{i,t\}$
    * i representing node index
    * t representing the feasible time index of the node
* Time-space arcs: in the form of $\{\{i, t\}, \{j, t^1\}\}$:
    * waiting arcs
    * serving arcs
    * moving arcs

### Variables

* 5 variables are included in the primary model:
    1. $z_{l,v}$: (binary) 1 if AMV $v$ travels on link $l$, 0 otherwise
    2. $y_{l,m}$: (binary) 1 if a platoon of length $m$ travels on link $l$, 0 otherwise
    3. $x_{i,v}$: (binary) 1 if AMV $v$ serves node $i$, 0 otherwise
    5. $e_{i,t,v}$: (integer) delivery amount of AMV $v$ for customer $i$ at time $t$
    7. $E_{l,r}$: (continuous) flow of pickup load of AMV $v$ on time-space arc $l$

### Objective Function

* three terms are considered in the objective function:
    1. total distance-related operation costs
    2. total trip duration for all vehicles
    3. total unserved requests

## Benchmark models
Two benchmark models are developed based on the primary model, each featuring a distinct platoon formation strategy that differs from that of the primary model. The key characteristics of the platooning principles for all three models are summarized below. 

* **Primary Model:** 
    Platoons can be formed and reconfigured at any physical location in the system over the time horizon.
* **Benchmark 1:** 
    Platoons will be formed at the central depot in the beginning and stay unchanged throughout the entire route (Jonas Hatzenbühler et al., 2023).
    * Two version to model benchmark 1 is provided. Benchmark1_1 is used since it is computationally faster than Benchmark1.
* **Benchmark 2:** 
    No platoons will be formed over the time horizon. Each MAV travels separately as in a conventional VRP setting.



## Getting Started

* The code should be compiled as a C++17 application.