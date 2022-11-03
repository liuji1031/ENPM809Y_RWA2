#ifndef __RW2_GROUP6_H__
#define __RW2_GROUP6_H__
#include <utility>
#include <array>
#include <unordered_map>
#include <string>

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
     * current location and north direction (m_curr_dir=0)
     */
    Mouse(): m_curr_loc_x{0},m_curr_loc_y{0},m_curr_dir{0},m_moves{0}{};
    /**
     * @brief mouse turns left by 90 deg
     */
    void turn_left();
    /**
     * @brief mouse turns right by 90 deg
     */
    void turn_right();
    /**
     * @brief turn the mouse by certain amount
     * 
     * @param dir_offset the amount to turn, -1 for left
     * 1 for right, 2 for turn around
     */
    void turn(int dir_offset);
    /**
     * @brief mouse moves forward by 1 
     * @param color the color to set for the maze when the mouse moves
     */
    void move_forward(char color);
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
     * -1 if data not available in the map, need to query simulator
     */
    int is_wall(int dir);

    private:
    /**
     * @brief m_wall stores the wall info in that cell
     * 
     */
    std::array<int,4> m_wall;
}; // class Cell
class Algorithm{
    public:
    /**
     * @brief Construct a new Algorithm object
     */
    Algorithm(): m_first_visit{0}{};
    /**
     * @brief initialize the maze, i.e., color outer walls
     */
    void init_maze();
    /**
     * @brief implements the left/right-wall-following algorithm for
     * the mouse
     * @param left_right_follow indicates which rule: left/right
     */
    void follow_wall(std::string left_right_follow);

    /**
     * @brief generates the goal position in the maze. needs to be
     * along the outer wall, other than (0,0)
     */
    void generate_goal();
    /**
     * @brief check if there is wall in a certain direction
     * 
     * @param flr the direction to be checked (front/left/right)
     * @return true if there is a wall
     * @return false if there is no wall
     */
    bool check_wall(int flr);
    
    /**
     * @brief direct the mouse back to the initial position
     */
    void return_to_init_loc();
    /**
     * @brief for every new location visited, store the number
     * of the move to first reach this location in an array
     */
    void update_first_vist();
    
    /**
     * @brief detect the wall at left/front/right and store the info
     * in the local map
     */
    void detect_wall_lfr(int dir,int x, int y);
    
    /**
     * @brief update the wall behind the mouse at the current location
     * in the maze whenever the mouse just moved forward
     * @param is_wall true for wall, false for no wall
     */
    void update_back_wall(bool is_wall);
    /**
     * @brief map from the relative direction (front, left, right)
     * to absolute direction (north, west, south, east) based on the
     * current direction of the mouse
     * @param d current direction of the mouse
     * @param flr relative direction (front/left.right)
     * @return absolute direction as an int (0-3, mapped from n/e/s/w)
     */
    static int calculate_dir(int dir, int flr);

    /**
     * @brief creates a mapping between nesw as chars to 0-3 as int
     * 
     * @param dir direction as char ('n','e','s','w')
     * @return const int direction as int (0,1,2,3)
     */
    static const int dir2int(char dir);
    /**
     * @brief creates a mapping between int (0-3) to char (n/e/s/w)
     */
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
     * walls. serves as a local map
     */
    std::array<std::array<Cell,m_maze_width>,m_maze_height> m_maze;
    /**
     * @brief the Mouse object that traverses the maze
     */
    Mouse m_mouse;
    /**
     * @brief m_first_visit stores the number of moves made when a particular 
     * location in the maze is first visited. E.g., the value of location (1,1) being 2 
     * means that location (1,1) is first visted on the second move. the array will
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
}; // class Algorithm

} // namespace rw2group6
#endif