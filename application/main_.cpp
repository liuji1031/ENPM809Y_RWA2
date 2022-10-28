#include "simulator\simulator.h"
#include "rw2_group6\rw2_group6.h"
#include <memory>

main(){
    auto solver = std::make_unique<rw2group6::Algorithm>();
    // initialize
    solver->init_maze();
    solver->generate_goal();    

    // get mouse to goal location
    solver->follow_wall("right");

    // go back to initial location
    solver->return_to_init_loc();
}
