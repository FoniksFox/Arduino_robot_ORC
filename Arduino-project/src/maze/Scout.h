#ifndef SCOUT_H
#define SCOUT_H

#include <iostream>
#include <vector>
#include <queue>
#include <set>
#include <map>
#include <algorithm>
#include <limits>
#include <tuple>
#include <cmath>
#include <string>
#include <unordered_map>
#include "pair_hash.h"

class Scout {
private:
    int x;                                          // Current x coordinate
    int y;                                          // Current y coordinate
    int maze_size;                                  // Size of the maze
    int direction;                                  // Current direction (0: North, 1: East, 2: South, 3: West)
    std::vector<std::vector<std::vector<int>>> knowledge_matrix; // 3D matrix to store wall knowledge
    std::vector<std::vector<bool>> visited;         // Matrix to track visited cells
    std::vector<std::string> path_history;          // History of moves
    bool reached_goal;                              // Flag to indicate if goal is reached
    bool fully_explored;                            // Flag to indicate if maze is fully explored
    int walls_discovered;                           // Count of walls discovered
    int total_possible_walls;                       // Total possible walls in the maze
    std::string next_move;                          // Next planned move
    std::tuple<int, int, int> current_state;        // Current state (y, x, direction)
    std::vector<std::tuple<int, int, int>> recent_positions; // Recent positions for loop detection
    std::map<std::tuple<int, int, int>, int> position_frequency; // Frequency of visiting specific positions
    std::set<std::pair<int, int>> deadend_cells;    // Set of identified dead-end cells
    int last_resort_direction;                      // Fallback direction when stuck
    std::vector<std::vector<std::vector<int>>> current_maze; // Current maze state

public:
    // Constructor
    Scout(int maze_size = 7);

    // Getter methods
    std::pair<int, int> get_position() const;
    int get_direction() const;
    int get_wall_status(const std::vector<std::vector<std::vector<int>>>& maze, int y, int x, int direction);
    double get_exploration_progress();
    bool is_fully_explored() const;
    const std::vector<std::vector<std::vector<int>>>& get_knowledge_matrix() const;
    const std::vector<std::vector<bool>>& get_visited() const;

    // Knowledge and exploration methods
    void update_forward_knowledge(const std::vector<std::vector<std::vector<int>>>& maze);
    bool is_visited(int y, int x) const;
    void check_exploration_status();
    std::pair<bool, int> check_for_loops();

    // Neighbor and navigation methods
    std::vector<std::pair<std::pair<int, int>, int>> get_unvisited_neighbors(
        const std::vector<std::vector<std::vector<int>>>& maze, 
        const std::pair<int, int>& position);
    int neighbor_potential(const std::pair<int, int>& position);
    std::string get_direction_to_target(const std::vector<std::vector<std::vector<int>>>& maze, 
        const std::pair<int, int>& target);
    std::pair<int, int> get_next_position();

    // Movement methods
    bool move_forward(const std::vector<std::vector<std::vector<int>>>& maze);
    void rotate_right();
    void rotate_left();
    std::string determine_next_move_dfs(const std::vector<std::vector<std::vector<int>>>& maze);
    std::string move_towards_unexplored(const std::vector<std::vector<std::vector<int>>>& maze);
    std::pair<int, int> get_position_in_direction(int dir);

    // Utility methods
    void update_wall_knowledge(int y, int x, int wall_direction, int status);
    void mark_visited(int y, int x);
    void reset();
};

#endif // SCOUT_H