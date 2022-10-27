#ifndef __RW2_GROUP6_H__
#define __RW2_GROUP6_H__
#include <utility>
#include <stack>
#include <array>
#include <unordered_map>
/**
 * @brief 
 * @author 
 * 
 */
#pragma once
namespace rw2group6{

class Mouse{
    public:
    /**
     * @brief Construct a new Mouse object with default (0,0)
     * current location 
     */
    Mouse(): m_curr_loc_x{0},m_curr_loc_y{0},m_curr_dir{0},m_moves{0}{
        // push initial position onto the stack
        std::array<int,3> init_pos{m_curr_loc_x,m_curr_loc_y,m_moves};
        m_loc_hist.push(init_pos);
    };
    /**
     * @brief mouse turns left by 90 deg
     */
    void turn_left();
    /**
     * @brief mouse turns right by 90 deg
     */
    void turn_right();
    /**
     * @brief mouse moves forward by 1 
     * @param color the color to set for the maze when the mouse moves
     */
    void move_forward(char color);
    /**
     * @brief put the current location and the number of moves
     * in the history stack
     */
    void record_curr_loc();
    /**
     * @brief Get the x location of mouse
     * 
     * @return returns m_curr_loc_x
     */
    int get_x();
    /**
     * @brief Get the y location of mouse
     * 
     * @return returns m_curr_loc_y
     */
    int get_y();
    /**
     * @brief Get the current direction of the mouse
     * 
     * @return returns m_curr_dir 
     */
    int get_dir();

    /**
     * @brief Get the number of moves
     * 
     * @return m_moves
     */
    int get_moves();

    private:
    /**
     * @brief m_curr_loc_x stores the current x location 
     */
    int m_curr_loc_x;
    /**
     * @brief m_curr_loc_y stores the current y location 
     */
    int m_curr_loc_y;
    /**
     * @brief m_moves is the number of moves the mouse has made
     */
    int m_moves;
    /**
     * @brief stores the currect direction of the robot
     * as a int: 'n','e','s','w' correspond to 0,1,2,3 respectively
     */
    int m_curr_dir;
    /**
     * @brief m_loc_hist is a stack storing the history
     * of explored locations in the maze. Each entry is 
     * a 1 by 3 array, with the first 2 elements being the
     * x and y coordinate of the explored location in the maze
     * and the 3rd element stores on which move the mouse arrived
     * at this location, e.g., arive at (0,1) after move 1 in the 
     * north direction
     */
    std::stack<std::array<int,3>> m_loc_hist; 
}; // class Mouse

class Cell{
    public:
    /**
     * @brief Construct the default Cell object with
     * no walls surrounding it
     */
    Cell():m_wall{-1,-1,-1,-1} {
    };
    /**
     * @brief Set the wall according to dir
     * 
     * @param dir the direction to which to set a wall
     * @param check_result based on check_result, set either 0 or 1
     * to be set
     */
    void set_wall(int dir,bool check_result);
    /**
     * @brief query whether a wall exists in a certain direction
     * 
     * @param dir direction of the wall to be queried
     * @return 0 or 1 if data exists in the map: 1 there is wall, 0 if no wall
     * @return -1 if data not available in the map, need to query simulator
     */
    int is_wall(int dir);

    private:
    std::array<int,4> m_wall;
};
class Algorithm{
    public:
    /**
     * @brief Construct a new Algorithm object
     */
    Algorithm(): m_first_visit{0}{

    };
    /**
     * @brief initialize the maze, set goal and direct mouse to 
     * the goal
     */
    void run();
    /**
     * @brief initialize the maze, i.e., color outer walls
     */
    void init_maze();
    /**
     * @brief implements the left-wall-following algorithm for
     * the mouse
     */
    void follow_wall_left();
    /**
     * @brief implements the right-wall-following algorithm for
     * the mouse
     */
    void follow_wall_right();
    /**
     * @brief generates the goal position in the maze. needs to be
     * along the outer wall, other than (0,0)
     */
    void generate_goal();
    /**
     * @brief check if there is wall in a certain direction
     * 
     * @param flr the direction to be checked
     * @return true if there is a wall
     * @return false if there is no wall
     */
    bool check_wall(int flr);
    
    /**
     * @brief direct the mouse to the initial position
     */
    void return_to_init_loc();
    /**
     * @brief check if it is visiting this location for 
     * the first time
     */
    void update_first_vist();
    
    /**
     * @brief update the wall status of the current cell and save
     * in the local map
     * 
     */
    void update_wall_curr_loc(int dir,int x, int y);
    
    /**
     * @brief map from the relative direction (front, left, right)
     * to absolute direction (north, west, south, east) based on the
     * current direction of the mouse
     * @param d current direction of the mouse
     * @param flr relative direction
     * @return char absolute direction in terms of the map
     */
    static int calculate_dir(int dir, int flr);

    static const int dir2int(char dir);
    static std::string int2dir;

    private:
    /**
     * @brief the maze size width is defined as a static const int 
     */
    int static const m_maze_width{16};
    /**
     * @brief the maze size height is defined as a static const int 
     */
    int static const m_maze_height{16};
    /**
     * @brief m_maze is an array of Cell objects that store the detected
     * walls
     */
    std::array<std::array<Cell,m_maze_width>,m_maze_height> m_maze;
    /**
     * @brief the Mouse object that traverses the maze
     */
    Mouse m_mouse;
    /**
     * @brief m_first_visit stores the when a particular location in the 
     * maze is first visited. The actual value stored is simply the earliest
     * move to reach this location. E.g., the value of (1,1) being 2 means 
     * that location (1,1) is first visted on the second move. the array will
     * be used to find the return path once the goal location is reached.
     */
    std::array<std::array<int,m_maze_width>,m_maze_height> m_first_visit;
    /**
     * @brief x coordinate of the goal location
     */
    int m_goal_x;
    /**
     * @brief y coordinate of the goal location
     */
    int m_goal_y;
    /**
     * @brief create a mapping between direction ("nesw")
     * and 0-3
     */

    

};

} // namespace rw2group6
#endif