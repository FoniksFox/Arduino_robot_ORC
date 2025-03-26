#include "PathFind.h"
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

PathFinder::PathFinder(int maze_size) : 
    maze_size(maze_size),
    current_target(std::make_pair(-1, -1)) {

    // Initialize knowledge matrix
    knowledge_matrix.resize(maze_size + 1, std::vector<std::vector<int>>(maze_size, std::vector<int>(4, -1)));

    // Initialize visited array
    visited.resize(maze_size + 1, std::vector<bool>(maze_size, false));
    }

    void PathFinder::set_knowledge_matrix(const std::vector<std::vector<std::vector<int>>>& matrix) {
    knowledge_matrix = matrix;
    }

    void PathFinder::set_visited(const std::vector<std::vector<bool>>& visit_matrix) {
    visited = visit_matrix;
    }

    void PathFinder::set_current_maze(const std::vector<std::vector<std::vector<int>>>& maze) {
    current_maze = maze;
    }

    void PathFinder::set_current_target(const std::pair<int, int>& target) {
    current_target = target;
    }

    std::pair<int, int> PathFinder::get_current_target() const {
    return current_target;
    }

    int PathFinder::get_wall_status(const std::vector<std::vector<std::vector<int>>>& maze, int y, int x, int direction) {
    if (maze.empty()) {
        if (current_maze.empty()) {
            return 1;
        }
        return current_maze[y][x][direction];
    }

    if (0 <= y && y < maze_size && 0 <= x && x < maze_size) {
        return maze[y][x][direction];
    }

    return 1;
    }

    int PathFinder::heuristic(const std::pair<int, int>& pos, const std::pair<int, int>& goal) {
    int y1 = pos.first;
    int x1 = pos.second;
    int y2 = goal.first;
    int x2 = goal.second;
    return std::abs(y1 - y2) + std::abs(x1 - x2);
    }

    std::vector<std::pair<int, int>> PathFinder::get_neighbors(const std::pair<int, int>& pos, 
                                                const std::vector<std::vector<std::vector<int>>>& maze) {
    int y = pos.first;
    int x = pos.second;
    std::vector<std::pair<int, int>> neighbors;

    std::vector<std::tuple<int, int, int>> directions = {
        std::make_tuple(0, -1, 0),  // North
        std::make_tuple(1, 0, 1),   // East
        std::make_tuple(2, 1, 0),   // South
        std::make_tuple(3, 0, -1)   // West
    };

    for (const auto& [dir_idx, dy, dx] : directions) {
        int ny = y + dy;
        int nx = x + dx;
        
        if (0 <= ny && ny < maze_size && 0 <= nx && nx < maze_size) {
            if (get_wall_status(maze, y, x, dir_idx) == 0) {
                neighbors.push_back(std::make_pair(ny, nx));
            }
        }
    }

    return neighbors;
    }

    std::vector<std::pair<int, int>> PathFinder::astar_search(const std::pair<int, int>& start, 
                                                const std::pair<int, int>& goal,
                                                const std::vector<std::vector<std::vector<int>>>& maze) {
    // Using sets for the open and closed sets
    std::set<std::pair<int, int>> open_set;
    std::set<std::pair<int, int>> closed_set;

    // Maps to store g_score and f_score
    std::unordered_map<std::pair<int, int>, int, PairHash> g_score;
    std::unordered_map<std::pair<int, int>, int, PairHash> f_score;

    // Map to store the path
    std::unordered_map<std::pair<int, int>, std::pair<int, int>, PairHash> came_from;

    open_set.insert(start);
    g_score[start] = 0;
    f_score[start] = heuristic(start, goal);

    while (!open_set.empty()) {
        // Find the node with the lowest f_score
        std::pair<int, int> current = *open_set.begin();
        for (const auto& pos : open_set) {
            if (f_score.find(pos) != f_score.end() && 
                (f_score.find(current) == f_score.end() || f_score[pos] < f_score[current])) {
                current = pos;
            }
        }
        
        // Check if we've reached the goal
        if (current == goal) {
            std::vector<std::pair<int, int>> path;
            while (came_from.find(current) != came_from.end()) {
                path.push_back(current);
                current = came_from[current];
            }
            path.push_back(start);
            std::reverse(path.begin(), path.end());
            return path;
        }
        
        open_set.erase(current);
        closed_set.insert(current);
        
        for (const auto& neighbor : get_neighbors(current, maze)) {
            if (closed_set.find(neighbor) != closed_set.end()) {
                continue;
            }
            
            int tentative_g_score = g_score[current] + 1;
            
            if (open_set.find(neighbor) == open_set.end()) {
                open_set.insert(neighbor);
            } else if (g_score.find(neighbor) != g_score.end() && tentative_g_score >= g_score[neighbor]) {
                continue;
            }
            
            came_from[neighbor] = current;
            g_score[neighbor] = tentative_g_score;
            f_score[neighbor] = g_score[neighbor] + heuristic(neighbor, goal);
        }
    }

    // Return empty path if no path found
    return std::vector<std::pair<int, int>>();
    }

    std::string PathFinder::get_next_move(const std::pair<int, int>& current_pos, int current_direction, 
                            const std::pair<int, int>& goal, 
                            const std::vector<std::vector<std::vector<int>>>& maze) {
    // Handle special case for goal with negative y-coordinate
    std::pair<int, int> adjusted_goal = goal;
    if (goal.first < 0) {
        adjusted_goal = std::make_pair(0, goal.second);
        
        if (current_pos == adjusted_goal) {
            if (current_direction != 0) {
                int clockwise_diff = (0 - current_direction + 4) % 4;
                if (clockwise_diff == 1) {
                    return "right";
                } else if (clockwise_diff == 3) {
                    return "left";
                } else {
                    return "right";
                }
            } else {
                return "forward";
            }
        }
        
        adjusted_goal = goal;
    }

    // Check if we're at the goal
    if (current_pos == goal) {
        if (goal == std::make_pair(0, maze_size / 2)) {
            if (current_direction != 0) {
                int clockwise_diff = (0 - current_direction + 4) % 4;
                if (clockwise_diff == 1) {
                    return "right";
                } else if (clockwise_diff == 3) {
                    return "left";
                } else {
                    return "right";
                }
            }
        }
        return "";
    }

    // Find path to goal
    std::vector<std::pair<int, int>> path = astar_search(current_pos, goal, maze);

    if (path.empty() || path.size() < 2) {
        return "";
    }

    std::pair<int, int> next_pos = path[1];

    int ny = next_pos.first;
    int nx = next_pos.second;
    int y = current_pos.first;
    int x = current_pos.second;

    int target_direction;
    if (ny < y) {
        target_direction = 0;  // North
    } else if (nx > x) {
        target_direction = 1;  // East
    } else if (ny > y) {
        target_direction = 2;  // South
    } else {
        target_direction = 3;  // West
    }

    if (current_direction == target_direction) {
        return "forward";
    }

    int clockwise_diff = (target_direction - current_direction + 4) % 4;

    if (clockwise_diff == 1) {
        return "right";
    } else if (clockwise_diff == 3) {
        return "left";
    } else {
        return "right";  // Turn right twice for 180 degrees
    }
    }

    std::string PathFinder::find_path_to_exit(const std::pair<int, int>& current_pos, int current_direction,
                                const std::vector<std::vector<std::vector<int>>>& maze) {
    std::pair<int, int> exit_pos = std::make_pair(0, maze_size / 2);
    return get_next_move(current_pos, current_direction, exit_pos, maze);
    }

    std::string PathFinder::find_path_to_nearest_unexplored(const std::pair<int, int>& current_pos, int current_direction,
                                            const std::vector<std::vector<std::vector<int>>>& maze) {
    // Find the closest unexplored cell
    std::pair<int, int> closest_unexplored(-1, -1);
    int min_distance = std::numeric_limits<int>::max();

    for (int i = 0; i < maze_size; i++) {
        for (int j = 0; j < maze_size; j++) {
            if (!visited[i][j]) {
                int distance = std::abs(i - current_pos.first) + std::abs(j - current_pos.second);
                if (distance < min_distance) {
                    // Check if this cell is reachable
                    std::vector<std::pair<int, int>> path = astar_search(current_pos, std::make_pair(i, j), maze);
                    if (!path.empty()) {
                        min_distance = distance;
                        closest_unexplored = std::make_pair(i, j);
                    }
                }
            }
        }
    }

    if (closest_unexplored.first != -1) {
        current_target = closest_unexplored;
        return get_next_move(current_pos, current_direction, current_target, maze);
    }

    // If no unexplored reachable cells, head to exit
    current_target = std::make_pair(0, maze_size / 2);
    return get_next_move(current_pos, current_direction, current_target, maze);
    }

    std::vector<std::pair<int, int>> PathFinder::get_accessible_cells(const std::pair<int, int>& current_pos,
                                                const std::vector<std::vector<std::vector<int>>>& maze) {
    std::vector<std::pair<int, int>> accessible;
    std::vector<std::vector<bool>> visited_check(maze_size, std::vector<bool>(maze_size, false));

    std::queue<std::pair<int, int>> queue;
    queue.push(current_pos);
    visited_check[current_pos.first][current_pos.second] = true;

    while (!queue.empty()) {
        auto [cell_y, cell_x] = queue.front();
        queue.pop();
        
        accessible.push_back(std::make_pair(cell_y, cell_x));
        
        std::vector<std::tuple<int, int, int>> directions = {
            std::make_tuple(0, -1, 0),  // North
            std::make_tuple(1, 0, 1),   // East
            std::make_tuple(2, 1, 0),   // South
            std::make_tuple(3, 0, -1)   // West
        };
        
        for (const auto& [d, dy, dx] : directions) {
            int ny = cell_y + dy;
            int nx = cell_x + dx;

            if (0 <= ny && ny < maze_size && 0 <= nx && nx < maze_size) {
                if (!visited_check[ny][nx]) {
                    if (get_wall_status(maze, cell_y, cell_x, d) == 0) {
                        visited_check[ny][nx] = true;
                        queue.push(std::make_pair(ny, nx));
                    }
                }
            }
        }
    }

    return accessible; 
    }

    bool PathFinder::check_fully_explored(const std::pair<int, int>& current_pos,
                        const std::vector<std::vector<std::vector<int>>>& maze) {
    int visited_count = 0;
    for (int i = 0; i < maze_size; i++) {
        for (int j = 0; j < maze_size; j++) {
            if (visited[i][j]) {
                visited_count++;
            }
        }
    }

    int total_cells = maze_size * maze_size;

    if (visited_count == total_cells) {
        return true;
    }

    auto accessible_cells = get_accessible_cells(current_pos, maze);

    bool all_accessible_visited = true;
    for (const auto& [cell_y, cell_x] : accessible_cells) {
        if (!visited[cell_y][cell_x]) {
            all_accessible_visited = false;
            break;
        }
    }

    return all_accessible_visited;
    }

    std::vector<std::pair<std::pair<int, int>, int>> PathFinder::get_unvisited_neighbors(
        const std::vector<std::vector<std::vector<int>>>& maze, 
        const std::pair<int, int>& position) {

    int y = position.first;
    int x = position.second;
    std::vector<std::pair<std::pair<int, int>, int>> unvisited;

    std::vector<std::tuple<int, int, int>> directions = {
        std::make_tuple(0, -1, 0),  // North
        std::make_tuple(1, 0, 1),   // East
        std::make_tuple(2, 1, 0),   // South
        std::make_tuple(3, 0, -1)   // West
    };

    for (const auto& [dir_idx, dy, dx] : directions) {
        int ny = y + dy;
        int nx = x + dx;
        
        if (0 <= ny && ny < maze_size && 0 <= nx && nx < maze_size) {
            if (get_wall_status(maze, y, x, dir_idx) == 0) {
                if (!visited[ny][nx]) {
                    unvisited.push_back(std::make_pair(std::make_pair(ny, nx), dir_idx));
                }
            }
        }
    }

    if (!unvisited.empty()) {
        std::sort(unvisited.begin(), unvisited.end(), 
        [this](const std::pair<std::pair<int, int>, int>& a, 
            const std::pair<std::pair<int, int>, int>& b) {
            return this->neighbor_potential(a.first) > this->neighbor_potential(b.first);
        });
    }

    return unvisited;
    }

    int PathFinder::neighbor_potential(const std::pair<int, int>& position) {
    int y = position.first;
    int x = position.second;
    int potential = 0;

    std::vector<std::pair<int, int>> directions = {
        std::make_pair(y-1, x),  // North
        std::make_pair(y, x+1),  // East
        std::make_pair(y+1, x),  // South
        std::make_pair(y, x-1)   // West
    };

    for (const auto& [ny, nx] : directions) {
        if (0 <= ny && ny < maze_size && 0 <= nx && nx < maze_size) {
            if (!visited[ny][nx]) {
                potential++;
            }
        }
    }

    return potential;
    }

    std::string PathFinder::optimize_exploration(const std::pair<int, int>& current_pos, int current_direction,
                            const std::vector<std::vector<std::vector<int>>>& maze) {
    // Find all unvisited cells
    std::vector<std::pair<int, int>> unvisited;
    for (int i = 0; i < maze_size; i++) {
        for (int j = 0; j < maze_size; j++) {
            if (!visited[i][j]) {
                unvisited.push_back(std::make_pair(i, j));
            }
        }
    }

    // If all cells are visited, head to the exit
    if (unvisited.empty()) {
        std::pair<int, int> exit_pos = std::make_pair(0, maze_size / 2);
        if (current_pos == exit_pos && current_direction != 0) {
            int clockwise_diff = (0 - current_direction + 4) % 4;
            if (clockwise_diff == 1) {
                return "right";
            } else if (clockwise_diff == 3) {
                return "left";
            } else {
                return "right";
            }
        }
        return get_next_move(current_pos, current_direction, exit_pos, maze);
    }

    // Find the closest unvisited cell
    std::pair<int, int> closest;
    int min_distance = std::numeric_limits<int>::max();

    for (const auto& cell : unvisited) {
        std::vector<std::pair<int, int>> path = astar_search(current_pos, cell, maze);
        if (path.empty()) {
            continue;
        }
        
        int distance = path.size() - 1;
        if (distance < min_distance) {
            min_distance = distance;
            closest = cell;
        }
    }

    // If no path found to any unvisited cell, go to exit
    if (min_distance == std::numeric_limits<int>::max()) {
        std::pair<int, int> exit_pos = std::make_pair(0, maze_size / 2);
        return get_next_move(current_pos, current_direction, exit_pos, maze);
    }

    return get_next_move(current_pos, current_direction, closest, maze);
    }

    double PathFinder::get_exploration_progress() {
    int visited_count = 0;
    for (int i = 0; i < maze_size; i++) {
        for (int j = 0; j < maze_size; j++) {
            if (visited[i][j]) {
                visited_count++;
            }
        }
    }
    int total_cells = maze_size * maze_size;

    return (visited_count / static_cast<double>(total_cells)) * 100.0;
    }

    void PathFinder::mark_visited(int y, int x) {
    if (0 <= y && y < maze_size && 0 <= x && x < maze_size) {
        visited[y][x] = true;
    }
    }

    void PathFinder::update_wall_knowledge(int y, int x, int wall_direction, int status) {
    // Update the knowledge matrix with the wall status
    if (0 <= y && y < maze_size && 0 <= x && x < maze_size && 0 <= wall_direction && wall_direction < 4) {
        knowledge_matrix[y][x][wall_direction] = status;
        
        // Update the opposite wall for the adjacent cell
        int opposite_direction = (wall_direction + 2) % 4;
        int adjacent_y = y;
        int adjacent_x = x;
        
        // Calculate adjacent cell coordinates
        if (wall_direction == 0) adjacent_y--; // North
        else if (wall_direction == 1) adjacent_x++; // East
        else if (wall_direction == 2) adjacent_y++; // South
        else if (wall_direction == 3) adjacent_x--; // West
        
        // Update adjacent cell if it's within bounds
        if (0 <= adjacent_y && adjacent_y < maze_size && 0 <= adjacent_x && adjacent_x < maze_size) {
            knowledge_matrix[adjacent_y][adjacent_x][opposite_direction] = status;
        }
    }
    }

    int PathFinder::get_direction_to_target(const std::pair<int, int>& current_pos, const std::pair<int, int>& target_pos) {
    if (target_pos.first == -1) {
        return 0; // Default direction
    }

    int y = current_pos.first;
    int x = current_pos.second;
    int target_y = target_pos.first;
    int target_x = target_pos.second;

    if (target_y < y) {
        return 0; // North
    } else if (target_x > x) {
        return 1; // East
    } else if (target_y > y) {
        return 2; // South
    } else {
        return 3; // West
    }
}