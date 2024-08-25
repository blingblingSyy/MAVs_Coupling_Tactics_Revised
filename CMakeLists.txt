cmake_minimum_required(VERSION 3.12)
project(MAVs_CD_PF) # AMVs_Couple_Decouple_Passenger_Freight

# Set the path to your Gurobi installation
set(GUROBI_HOME "C:/gurobi1002/win64")

# Add the Gurobi include directory
include_directories("${GUROBI_HOME}/include")

# Add the Gurobi library directory
link_directories("${GUROBI_HOME}/lib")

# Add your source files
add_executable(main main.cpp utility.cpp initialize.cpp model.cpp mainmodel.cpp benchmark1.cpp benchmark1_1.cpp benchmark2.cpp)

# Link against the Gurobi library
target_link_libraries(main "${GUROBI_LIB_DIR}\\gurobi100.lib" "${GUROBI_LIB_DIR}\\gurobi_c++mdd2017.lib")
