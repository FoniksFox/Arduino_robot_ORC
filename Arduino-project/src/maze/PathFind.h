#ifndef PATH_FINDER_H
#define PATH_FINDER_H

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

class PathFinder {
private:
    int maze_size;
    std::vector<std::vector<std::vector<int>>> knowledge_matrix;
    std::vector<std::vector<bool>> visited;
    std::pair<int, int> current_target;
    std::vector<std::vector<std::vector<int>>> current_maze;

public:
    PathFinder(int maze_size = 7);
    
    void set_knowledge_matrix(const std::vector<std::vector<std::vector<int>>>& matrix);
    void set_visited(const std::vector<std::vector<bool>>& visit_matrix);
    void set_current_maze(const std::vector<std::vector<std::vector<int>>>& maze);
    void set_current_target(const std::pair<int, int>& target);
    
    std::pair<int, int> get_current_target() const;
    
    int get_wall_status(const std::vector<std::vector<std::vector<int>>>& maze, int y, int x, int direction);
    
    int heuristic(const std::pair<int, int>& pos, const std::pair<int, int>& goal);
    
    std::vector<std::pair<int, int>> get_neighbors(const std::pair<int, int>& pos, 
                                                   const std::vector<std::vector<std::vector<int>>>& maze);
    
    std::vector<std::pair<int, int>> astar_search(const std::pair<int, int>& start, 
                                                  const std::pair<int, int>& goal,
                                                  const std::vector<std::vector<std::vector<int>>>& maze);
    
    std::string get_next_move(const std::pair<int, int>& current_pos, int current_direction, 
                               const std::pair<int, int>& goal, 
                               const std::vector<std::vector<std::vector<int>>>& maze);
    
    std::string find_path_to_exit(const std::pair<int, int>& current_pos, int current_direction,
                                   const std::vector<std::vector<std::vector<int>>>& maze);
    
    std::string find_path_to_nearest_unexplored(const std::pair<int, int>& current_pos, int current_direction,
                                                 const std::vector<std::vector<std::vector<int>>>& maze);
    
    std::vector<std::pair<int, int>> get_accessible_cells(const std::pair<int, int>& current_pos,
                                                          const std::vector<std::vector<std::vector<int>>>& maze);
    
    bool check_fully_explored(const std::pair<int, int>& current_pos,
                               const std::vector<std::vector<std::vector<int>>>& maze);
    
    std::vector<std::pair<std::pair<int, int>, int>> get_unvisited_neighbors(
            const std::vector<std::vector<std::vector<int>>>& maze, 
            const std::pair<int, int>& position);
    
    int neighbor_potential(const std::pair<int, int>& position);
    
    std::string optimize_exploration(const std::pair<int, int>& current_pos, int current_direction,
                                     const std::vector<std::vector<std::vector<int>>>& maze);
    
    double get_exploration_progress();
    
    void mark_visited(int y, int x);
    
    void update_wall_knowledge(int y, int x, int wall_direction, int status);
    
    int get_direction_to_target(const std::pair<int, int>& current_pos, const std::pair<int, int>& target_pos);
};

#endif // PATH_FINDER_H