#include "simulator\simulator.h"
#include "rw2_group6\rw2_group6.h"

main(){
    rw2group6::Algorithm solver;
    // initialize
    solver.init_maze();
    solver.generate_goal();    

    // get mouse to goal location
    solver.follow_wall_left();

    // go back to initial location
    solver.return_to_init_loc();
}
