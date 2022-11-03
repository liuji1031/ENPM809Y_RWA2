#include "..\simulator\simulator.h"
#include "rw2_group6.h"
#include <array>
#include <cstdlib>
#include <random> 
#include <iostream>
#include <ctime>
#include <string>
void rw2group6::Mouse::turn(int dir_offset){
    if(dir_offset==-1)turn_left();
    else if(dir_offset==1)turn_right();
    else if(dir_offset==2){turn_right();turn_right();}
    else if(dir_offset==0){//do nothing 
    }
}
void rw2group6::Mouse::turn_left(){
    m_curr_dir = rw2group6::Algorithm::calculate_dir(m_curr_dir,-1);
    Simulator::turnLeft();
    // std::cerr << m_curr_dir << " mouse turned left\n";
}
void rw2group6::Mouse::turn_right(){
    m_curr_dir = rw2group6::Algorithm::calculate_dir(m_curr_dir,1);
    Simulator::turnRight();
    // std::cerr << m_curr_dir << " mouse turned right\n";
}

void rw2group6::Mouse::move_forward(char color){
    // assume the move is valid and will not crash
    // into walls
    if(m_curr_dir == Algorithm::dir2int('n'))m_curr_loc_y++;
    else if(m_curr_dir == Algorithm::dir2int('e'))m_curr_loc_x++;
    else if(m_curr_dir == Algorithm::dir2int('s'))m_curr_loc_y--;
    else if(m_curr_dir == Algorithm::dir2int('w'))m_curr_loc_x--;
    m_moves++;
    Simulator::moveForward();
    Simulator::setColor(m_curr_loc_x,m_curr_loc_y,color);
    // std::cerr << m_curr_dir << " mouse moved forward\n";
}

int rw2group6::Mouse::get_dir(){
    return m_curr_dir;
}
int rw2group6::Mouse::get_x(){
    return m_curr_loc_x;
}
int rw2group6::Mouse::get_y(){
    return m_curr_loc_y;
}

int rw2group6::Mouse::get_moves(){
    return m_moves;
}
int rw2group6::Cell::is_wall(int dir){
    return m_wall.at(dir);
}


void rw2group6::Cell::set_wall(int dir, bool check_result){
    // set the wall in the corresponding direction according to
    // check_result
    m_wall.at(dir)=static_cast<int>(check_result);
}
void rw2group6::Algorithm::init_maze(){
    Simulator::setColor(0,0,'c');
    m_first_visit[0][0]=0; // move zero  
    for(int i{0};i<m_maze_width;i++){
        // bottom wall
        m_maze[0][i].set_wall(dir2int('s'),1);
        Simulator::setWall(i,0,'s');
        // top wall
        m_maze[m_maze_height-1][i].set_wall(dir2int('n'),1);
        Simulator::setWall(i,m_maze_height-1,'n');
    }
    for(int i{0};i<m_maze_height;i++){
        // left wall
        m_maze[i][0].set_wall(dir2int('w'),1);
        Simulator::setWall(0,i,'w');
        // right wall
        m_maze[i][m_maze_width-1].set_wall(dir2int('e'),1);
        Simulator::setWall(m_maze_width-1,i,'e');
    }
}

const int rw2group6::Algorithm::dir2int(char dir){
        static std::unordered_map<char,int> dir2intmap{{'n',0},{'e',1},{'s',2},{'w',3}};
        return dir2intmap[dir];
}
std::string rw2group6::Algorithm::int2dir = "nesw";

void rw2group6::Algorithm::generate_goal(){
    std::random_device rd; // obtain a random number from hardware
    std::mt19937 gen(rd()); // seed the generator
    std::uniform_int_distribution<> distr(0, m_maze_width-1); // define the range
    std::uniform_int_distribution<> distr2(0, 3); // define the range
    while(1){
        // first decide which wall to put the goal
        int pos{distr(gen)};
        std::srand(std::time(0));
        int i{distr2(gen)};
        if(i==0){ // left wall
            if(pos!=0){ // not at origin
                m_goal_x = 0;
                m_goal_y = pos;
                break;
            }
        }
        if(i==1){ // top wall
            m_goal_x = pos;
            m_goal_y = m_maze_height-1;
            break;
        }
        if(i==2){ // right wall
            m_goal_x = m_maze_width-1;
            m_goal_y = pos;
            break;
        }
        if(i==3){ // bottom wall
            if(pos!=0){
                m_goal_x = pos;
                m_goal_y = 0;
                break;
            }
        }
    }
    // color the goal position
    Simulator::setColor(m_goal_x,m_goal_y,'w');
}
int rw2group6::Algorithm::calculate_dir(int dir,int flr){
    int d_{dir + flr};
    if(d_<0)d_+=4;
    if(d_>3)d_-=4;
    return d_;
}
void rw2group6::Algorithm::update_first_vist(){
    int x{m_mouse.get_x()};
    int y{m_mouse.get_y()};
    if(m_first_visit[y][x]==0){ //  not visited
        m_first_visit[y][x] = m_mouse.get_moves();
    }
}
bool rw2group6::Algorithm::check_wall(int lfr){
    // check if wall exists in the dir direction
    // get the current location of the mouse 
    int x{m_mouse.get_x()};
    int y{m_mouse.get_y()};
    int d{m_mouse.get_dir()};
    // calculate absolute direction
    int d_{Algorithm::calculate_dir(d,lfr)};
    int check_{m_maze[y][x].is_wall(d_)};
    bool check_result{};
    if(check_==1) {
        check_result = true;
    }else if(check_==0) {
        check_result = false;
    }
    return check_result;
}

void rw2group6::Algorithm::detect_wall_lfr(int curr_dir, int x, int y){
    // this function ensures the wall information for the 
    // current cell is stored in the local map for the left, 
    // front and right direction
    std::array<int,3> dirs{-1,0,1}; // left, front, right
    for(int d : dirs){ // go through all 3 directions
        // see if the local map has the information
        int d_{calculate_dir(curr_dir,d)}; 
        bool check_result{}; // check if local map has the information
        
        if(m_maze[y][x].is_wall(d_)==-1){ // no data availabe in the local map, query simulator
            switch (d)
            {
            case 0:
                check_result = Simulator::wallFront();
                break;
            case -1:
                check_result = Simulator::wallLeft();
                break;
            case 1:
                check_result = Simulator::wallRight();
                break;
            }
            // save check result in cell (local map)
            m_maze[y][x].set_wall(d_,check_result);
            // set wall color in the simulation
            if(check_result==1)
                Simulator::setWall(x,y,int2dir.at(d_));
        }
    }
}
void rw2group6::Algorithm::follow_wall(std::string left_right_follow){
    int do_move{};
    char color{'c'}; // the color to fill the path taken
    int first_dir{}; // the first direction to check changes based on left/right wall following
    if(left_right_follow=="left")first_dir = -1;
    else first_dir = 1;
    // detect left, front and right wall at the (0,0) location
    detect_wall_lfr(m_mouse.get_dir(),m_mouse.get_x(),m_mouse.get_y());
    while(m_mouse.get_x()!=m_goal_x || m_mouse.get_y()!=m_goal_y){
        do_move = 1; // flag indicating whether to move to a new location, default is to move
        // store wall information in the local map first
        int dir{m_mouse.get_dir()};
        // update first vist record
        update_first_vist();
        if(!check_wall(first_dir)){ 
            m_mouse.turn(first_dir);
        }else if(!check_wall(0)){ // 0  is front
            // do nothing
        }else if(!check_wall(-first_dir)){  // check the opposite direction of first_dir
            m_mouse.turn(-first_dir);
        }else{
            // turn around
            m_mouse.turn(2);
            do_move = 0;
        }
        if(do_move){
            m_mouse.move_forward(color);
            // just move to the new location, we can immediately update the 
            // wall behind us, because there can't be any wall 
            update_back_wall(false);
            // at a new location, detect any wall to the left, front and right 
            // if the information is not in the local map
            detect_wall_lfr(m_mouse.get_dir(),m_mouse.get_x(),m_mouse.get_y());
        }
    }
    update_first_vist();
}

void rw2group6::Algorithm::update_back_wall(bool is_wall){
    int dir_offset{2}; // the direction offset for turning around
    m_maze[m_mouse.get_y()][m_mouse.get_x()].set_wall(calculate_dir(m_mouse.get_dir(),dir_offset),is_wall);
}

void rw2group6::Algorithm::return_to_init_loc(){
    // return to (0,0)
    // for each current location, look through the 4 connected locations
    // among the 4 locations, find the one that's visted before and has the
    // earliest visit and also reachable 
    // first clear all color
    Simulator::clearAllColor();
    int x{m_mouse.get_x()};
    int y{m_mouse.get_y()};
    std::array<int,4> dir_x{0,1,0,-1};
    std::array<int,4> dir_y{1,0,-1,0};
    std::array<int,3> lfr{-1,0,1};
    // first turn around
    m_mouse.turn(2); 
    char color = 'o';
    Simulator::setColor(x,y,color);
    int early_visit{};
    int dir_next{};
    while(x!=0 || y!=0){
        early_visit = -1;
        for(int d : lfr){
            if(check_wall(d)==1)continue; // 1 means there is a wall
            // calculate direction relative to the current direction of the mouse
            int d_{calculate_dir(m_mouse.get_dir(),d)}; 
            // compute x, y coordinate in that direction
            int x_{x+dir_x.at(d_)};
            int y_{y+dir_y.at(d_)};
            if(x_>=0 && x_<m_maze_width && y_>=0 && y_<m_maze_height){ // if within maze
                if(y_==0 && x_==0 ){ // if found (0,0), break out of the loop
                    dir_next = d;
                    break;
                }
                if(m_first_visit[y_][x_]==0)continue; // not visited
                if(early_visit==-1){ // find the earliest visit among all adjacent and reachable locations
                    early_visit=m_first_visit[y_][x_];
                    dir_next = d;
                } else {
                    if(m_first_visit[y_][x_]<early_visit){
                        early_visit = m_first_visit[y_][x_];
                        dir_next = d;
                    }
                }
            }
        }
        // now go according to dir_next
        m_mouse.turn(dir_next);
        m_mouse.move_forward(color);
        // get the new location
        x = m_mouse.get_x();
        y = m_mouse.get_y();
    }
}
