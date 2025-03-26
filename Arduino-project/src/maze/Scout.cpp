#include "Scout.h"
#include <algorithm>
#include <queue>
#include <cmath>

Scout::Scout(int maze_size) : 
    maze_size(maze_size),
    x(maze_size / 2),
    y(maze_size - 1),
    direction(0),
    reached_goal(false),
    fully_explored(false),
    walls_discovered(0),
    last_resort_direction(0) {
    
    total_possible_walls = 2 * maze_size * (maze_size - 1) + 4 * maze_size - 4;
    
    knowledge_matrix.resize(maze_size + 1, std::vector<std::vector<int>>(maze_size, std::vector<int>(4, -1)));
    
    visited.resize(maze_size + 1, std::vector<bool>(maze_size, false));
    visited[y][x] = true;
    
    // Initialize current state
    current_state = std::make_tuple(y, x, direction);
}

// Get current position
std::pair<int, int> Scout::get_position() const {
    return std::make_pair(y, x);
}

// Get current direction
int Scout::get_direction() const {
    return direction;
}

// Get wall status from maze knowledge
int Scout::get_wall_status(const std::vector<std::vector<std::vector<int>>>& maze, int wall_y, int wall_x, int direction) {
    if (maze.empty()) {
        if (current_maze.empty()) {
            return 1; // Assume wall if no information
        }
        return get_wall_status(current_maze, wall_y, wall_x, direction);
    }
    
    if (0 <= wall_y && wall_y < maze_size && 0 <= wall_x && wall_x < maze_size) {
        return maze[wall_y][wall_x][direction];
    }
    
    return 1; // Assume wall at boundaries
}

// Update knowledge of the current cell's walls
void Scout::update_forward_knowledge(const std::vector<std::vector<std::vector<int>>>& maze) {
    current_maze = maze;
    
    if (0 <= y && y < maze_size && 0 <= x && x < maze_size) {
        std::vector<int> current_walls = maze[y][x];
        
        for (int i = 0; i < 4; i++) {
            if (knowledge_matrix[y][x][i] == -1) {
                knowledge_matrix[y][x][i] = current_walls[i];
                if (current_walls[i] == 1) {
                    walls_discovered++;
                }
            }
        }
        
        // Check if maze is fully explored
        check_exploration_status();
    }
}

// Check if a cell is visited
bool Scout::is_visited(int visit_y, int visit_x) const {
    if (0 <= visit_y && visit_y < maze_size && 0 <= visit_x && visit_x < maze_size) {
        return visited[visit_y][visit_x];
    }
    return false;
}

// Check exploration status
void Scout::check_exploration_status() {
    int unvisited_cells = 0;
    for (int i = 0; i < maze_size; i++) {
        for (int j = 0; j < maze_size; j++) {
            if (!visited[i][j]) {
                unvisited_cells++;
            }
        }
    }
    
    if (unvisited_cells == 0) {
        bool all_walls_known = true;
        for (int y = 0; y < maze_size; y++) {
            for (int x = 0; x < maze_size; x++) {
                for (int i = 0; i < 4; i++) {
                    if (knowledge_matrix[y][x][i] == -1) {
                        all_walls_known = false;
                        break;
                    }
                }
                if (!all_walls_known) break;
            }
            if (!all_walls_known) break;
        }
        fully_explored = all_walls_known;
    }
}

// Check if we're stuck in a loop
std::pair<bool, int> Scout::check_for_loops() {
    current_state = std::make_tuple(y, x, direction);
    
    // Update position frequency
    if (position_frequency.find(current_state) != position_frequency.end()) {
        position_frequency[current_state]++;
    } else {
        position_frequency[current_state] = 1;
    }
    
    // Update recent positions history
    recent_positions.push_back(current_state);
    if (recent_positions.size() > 20) {
        recent_positions.erase(recent_positions.begin());
    }
    
    // Check if we're visiting this position too frequently
    if (position_frequency[current_state] > 3) {
        position_frequency[current_state] = 0;
        
        // Check if this is a dead end
        int walls_around = 0;
        for (int d = 0; d < 4; d++) {
            if (get_wall_status(current_maze, y, x, d) == 1) {
                walls_around++;
            }
        }
        
        if (walls_around >= 3) {
            deadend_cells.insert(std::make_pair(y, x));
        }
        
        // Find first available direction
        for (int d = 0; d < 4; d++) {
            if (get_wall_status(current_maze, y, x, d) == 0) {
                return std::make_pair(true, d);
            }
        }
        
        // Last resort if all directions have walls (shouldn't happen)
        last_resort_direction = (last_resort_direction + 1) % 4;
        return std::make_pair(true, last_resort_direction);
    }
    
    // Check for short loop patterns
    if (recent_positions.size() >= 6) {
        if (recent_positions[recent_positions.size() - 1] == recent_positions[recent_positions.size() - 3] &&
            recent_positions[recent_positions.size() - 2] == recent_positions[recent_positions.size() - 4] &&
            recent_positions[recent_positions.size() - 3] == recent_positions[recent_positions.size() - 5]) {
            
            // Break pattern by finding first available direction
            for (int d = 0; d < 4; d++) {
                if (get_wall_status(current_maze, y, x, d) == 0) {
                    return std::make_pair(true, d);
                }
            }
            
            // Turn around if no other option
            return std::make_pair(true, (direction + 2) % 4);
        }
    }
    
    return std::make_pair(false, -1);
}

// Get unvisited neighboring cells
std::vector<std::pair<std::pair<int, int>, int>> Scout::get_unvisited_neighbors(
        const std::vector<std::vector<std::vector<int>>>& maze, 
        const std::pair<int, int>& position) {
    
    current_maze = maze;
    
    int pos_y = position.first;
    int pos_x = position.second;
    std::vector<std::pair<std::pair<int, int>, int>> unvisited;
    
    std::vector<std::tuple<int, int, int>> directions = {
        std::make_tuple(0, -1, 0),  // North
        std::make_tuple(1, 0, 1),   // East
        std::make_tuple(2, 1, 0),   // South
        std::make_tuple(3, 0, -1)   // West
    };
    
    for (const auto& [dir_idx, dy, dx] : directions) {
        int ny = pos_y + dy;
        int nx = pos_x + dx;
        
        if (0 <= ny && ny < maze_size && 0 <= nx && nx < maze_size) {
            if (get_wall_status(maze, pos_y, pos_x, dir_idx) == 0) {
                if (!visited[ny][nx]) {
                    unvisited.push_back(std::make_pair(std::make_pair(ny, nx), dir_idx));
                }
            }
        }
    }
    
    // Sort unvisited neighbors by potential (how many unvisited cells they might lead to)
    if (!unvisited.empty()) {
        std::sort(unvisited.begin(), unvisited.end(), 
        [this](const std::pair<std::pair<int, int>, int>& a, 
            const std::pair<std::pair<int, int>, int>& b) {
            return this->neighbor_potential(a.first) > this->neighbor_potential(b.first);
        });
    }
    
    return unvisited;
}

// Calculate potential of a cell (how many unvisited neighbors it has)
int Scout::neighbor_potential(const std::pair<int, int>& position) {
    int pos_y = position.first;
    int pos_x = position.second;
    int potential = 0;
    
    std::vector<std::pair<int, int>> directions = {
        std::make_pair(pos_y-1, pos_x),  // North
        std::make_pair(pos_y, pos_x+1),  // East
        std::make_pair(pos_y+1, pos_x),  // South
        std::make_pair(pos_y, pos_x-1)   // West
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

// Move forward if possible
bool Scout::move_forward(const std::vector<std::vector<std::vector<int>>>& maze) {
    update_forward_knowledge(maze);
    
    // Check if we can move forward
    bool can_move_forward = false;
    if (y == maze_size) {
        // Starting position
        can_move_forward = true;
    } else if (y == 0 && x == maze_size / 2 && direction == 0) {
        // Exit position
        can_move_forward = true;
    } else if (0 <= y && y < maze_size && 0 <= x && x < maze_size) {
        can_move_forward = get_wall_status(maze, y, x, direction) == 0;
    }
    
    if (can_move_forward) {
        auto [next_y, next_x] = get_next_position();
        
        // Don't exit if not fully explored
        if (next_y < 0 && !fully_explored) {
            return false;
        }
        
        // Update position
        y = next_y;
        x = next_x;
        
        // Mark as visited if inside maze
        if (0 <= y && y < maze_size && 0 <= x && x < maze_size) {
            visited[y][x] = true;
        }
        
        path_history.push_back("forward");
        
        // Check if reached exit
        if (y == -1 && x == maze_size / 2 && fully_explored) {
            reached_goal = true;
        }
        
        return true;
    }
    
    return false;
}

// Rotate right
void Scout::rotate_right() {
    direction = (direction + 1) % 4;
    path_history.push_back("right");
}

// Rotate left
void Scout::rotate_left() {
    direction = (direction - 1 + 4) % 4;
    path_history.push_back("left");
}

// Determine next move using depth-first search
std::string Scout::determine_next_move_dfs(const std::vector<std::vector<std::vector<int>>>& maze) {
    current_maze = maze;
    
    // If we're at the exit and facing north, we're done
    if (y == 0 && x == maze_size / 2 && direction == 0) {
        reached_goal = true;
        return "forward";
    }
    
    // Check for unexplored neighbors
    std::vector<std::tuple<int, int, int, int>> directions = {
        std::make_tuple(0, -1, 0, 0), // North
        std::make_tuple(1, 0, 1, 1),  // East
        std::make_tuple(2, 1, 0, 2),  // South
        std::make_tuple(3, 0, -1, 3)  // West
    };
    
    // First, check if there's an unexplored cell we can move to
    for (const auto& [dir_idx, dy, dx, dir] : directions) {
        int ny = y + dy;
        int nx = x + dx;
        
        if (0 <= ny && ny < maze_size && 0 <= nx && nx < maze_size) {
            if (get_wall_status(maze, y, x, dir_idx) == 0 && !visited[ny][nx]) {
                // Rotate to face the direction of the unexplored cell
                int target_direction = dir;
                int clockwise_diff = (target_direction - direction + 4) % 4;
                
                if (clockwise_diff == 0) {
                    return "forward";
                } else if (clockwise_diff == 1) {
                    return "right";
                } else if (clockwise_diff == 3) {
                    return "left";
                } else {
                    return "right"; // Turn right twice (180 degrees)
                }
            }
        }
    }
    
    // If we reach here, all neighbors are explored
    // Use the move_towards_unexplored strategy
    return move_towards_unexplored(maze);
}

// Get direction to target
std::string Scout::get_direction_to_target(const std::vector<std::vector<std::vector<int>>>& maze, 
    const std::pair<int, int>& target) {
    // If already at target, no move needed
    if (std::make_pair(y, x) == target) {
        return "forward";
    }
    
    // Basic breadth-first search to find path
    std::queue<std::pair<int, int>> queue;
    std::vector<std::vector<bool>> visited_bfs(maze_size, std::vector<bool>(maze_size, false));
    std::vector<std::vector<std::pair<int, int>>> parent(maze_size, 
        std::vector<std::pair<int, int>>(maze_size, std::make_pair(-1, -1)));
    
    queue.push(std::make_pair(y, x));
    visited_bfs[y][x] = true;
    
    // Possible move directions: North, East, South, West
    std::vector<std::pair<int, int>> directions = {
        {-1, 0}, {0, 1}, {1, 0}, {0, -1}
    };
    
    while (!queue.empty()) {
        auto current = queue.front();
        queue.pop();
        
        // Found target
        if (current == target) {
            // Reconstruct path
            std::vector<std::pair<int, int>> path;
            while (current != std::make_pair(y, x)) {
                path.push_back(current);
                current = parent[current.first][current.second];
            }
            
            // If path is empty, we're already at target
            if (path.empty()) {
                return "forward";
            }
            
            // Get the first step from current position
            auto next_step = path.back();
            
            // Determine direction to move
            if (next_step.first < y) {
                // Move North
                int clockwise_diff = (0 - direction + 4) % 4;
                if (clockwise_diff == 0) return "forward";
                if (clockwise_diff == 1) return "right";
                if (clockwise_diff == 3) return "left";
                return "right"; // 180 degree turn
            }
            else if (next_step.first > y) {
                // Move South
                int clockwise_diff = (2 - direction + 4) % 4;
                if (clockwise_diff == 0) return "forward";
                if (clockwise_diff == 1) return "right";
                if (clockwise_diff == 3) return "left";
                return "right"; // 180 degree turn
            }
            else if (next_step.second < x) {
                // Move West
                int clockwise_diff = (3 - direction + 4) % 4;
                if (clockwise_diff == 0) return "forward";
                if (clockwise_diff == 1) return "right";
                if (clockwise_diff == 3) return "left";
                return "right"; // 180 degree turn
            }
            else if (next_step.second > x) {
                // Move East
                int clockwise_diff = (1 - direction + 4) % 4;
                if (clockwise_diff == 0) return "forward";
                if (clockwise_diff == 1) return "right";
                if (clockwise_diff == 3) return "left";
                return "right"; // 180 degree turn
            }
        }
        
        // Explore neighbors
        for (const auto& dir : directions) {
            int ny = current.first + dir.first;
            int nx = current.second + dir.second;
            
            // Check bounds and wall status
            if (ny >= 0 && ny < maze_size && nx >= 0 && nx < maze_size &&
                !visited_bfs[ny][nx] && 
                get_wall_status(maze, current.first, current.second, 
                    (dir.first == -1 ? 0 : (dir.first == 1 ? 2 : (dir.second == -1 ? 3 : 1))) ) == 0) {
                
                queue.push(std::make_pair(ny, nx));
                visited_bfs[ny][nx] = true;
                parent[ny][nx] = current;
            }
        }
    }
    
    // No path found
    return "right"; // Default to turning right if no path
}

// Move towards the closest unexplored cell
std::string Scout::move_towards_unexplored(const std::vector<std::vector<std::vector<int>>>& maze) {
    // Find any directions without walls
    std::vector<int> available_directions;
    for (int d = 0; d < 4; d++) {
        if (get_wall_status(maze, y, x, d) == 0) {
            available_directions.push_back(d);
        }
    }
    
    if (available_directions.empty()) {
        // No available directions (shouldn't happen)
        return "right";
    }
    
    // Choose a direction we haven't been often
    std::vector<std::pair<int, int>> direction_scores;
    for (int d : available_directions) {
        auto next_pos = get_position_in_direction(d);
        int ny = next_pos.first;
        int nx = next_pos.second;
        
        if (0 <= ny && ny < maze_size && 0 <= nx && nx < maze_size) {
            // Calculate a score based on visit frequency and dead end status
            int score = 100;
            
            // Lower score for visited cells
            if (visited[ny][nx]) {
                score -= 50;
            }
            
            // Lower score for dead ends
            if (deadend_cells.find(std::make_pair(ny, nx)) != deadend_cells.end()) {
                score -= 30;
            }
            
            // Lower score for frequently visited positions
            std::tuple<int, int, int> pos_state = std::make_tuple(ny, nx, d);
            if (position_frequency.find(pos_state) != position_frequency.end()) {
                score -= position_frequency[pos_state] * 10;
            }
            
            direction_scores.push_back(std::make_pair(score, d));
        } else {
            // Handle boundary cases
            direction_scores.push_back(std::make_pair(0, d));
        }
    }
    
    // Sort by scores (highest first)
    std::sort(direction_scores.begin(), direction_scores.end(), 
             [](const std::pair<int, int>& a, const std::pair<int, int>& b) {
                 return a.first > b.first;
             });
    
    // Use the highest scoring direction
    int best_direction = direction_scores[0].second;
    
    // Calculate how to turn to that direction
    int clockwise_diff = (best_direction - direction + 4) % 4;
    if (clockwise_diff == 0) {
        return "forward";
    } else if (clockwise_diff == 1) {
        return "right";
    } else if (clockwise_diff == 3) {
        return "left";
    } else {
        return "right"; // Turn right twice (180 degrees)
    }
}

// Get position if moving in specified direction
std::pair<int, int> Scout::get_position_in_direction(int dir) {
    int ny = y, nx = x;
    
    if (dir == 0) {
        ny = y - 1;  // North
    } else if (dir == 1) {
        nx = x + 1;  // East
    } else if (dir == 2) {
        ny = y + 1;  // South
    } else if (dir == 3) {
        nx = x - 1;  // West
    }
    
    return std::make_pair(ny, nx);
}

// Update wall knowledge
void Scout::update_wall_knowledge(int wall_y, int wall_x, int wall_direction, int status) {
    // Update the knowledge matrix with the wall status
    if (0 <= wall_y && wall_y < maze_size && 0 <= wall_x && wall_x < maze_size && 0 <= wall_direction && wall_direction < 4) {
        knowledge_matrix[wall_y][wall_x][wall_direction] = status;
        
        // Update the opposite wall for the adjacent cell
        int opposite_direction = (wall_direction + 2) % 4;
        int adjacent_y = wall_y;
        int adjacent_x = wall_x;
        
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

void Scout::mark_visited(int y, int x) {
    if (0 <= y && y < maze_size && 0 <= x && x < maze_size) {
        visited[y][x] = true;
    }
}

void Scout::reset() {
    x = maze_size / 2;
    y = maze_size - 1;
    direction = 0;
    reached_goal = false;
    fully_explored = false;
    walls_discovered = 0;
    last_resort_direction = 0;

    knowledge_matrix.clear();
    knowledge_matrix.resize(maze_size + 1, std::vector<std::vector<int>>(maze_size, std::vector<int>(4, -1)));

    visited.clear();
    visited.resize(maze_size + 1, std::vector<bool>(maze_size, false));
    visited[y][x] = true;

    path_history.clear();
    recent_positions.clear();
    position_frequency.clear();
    deadend_cells.clear();
    current_state = std::make_tuple(y, x, direction);
}

// Methods from header that need implementation

double Scout::get_exploration_progress() {
    int total_cells = maze_size * maze_size;
    int visited_cells = 0;

    for (int i = 0; i < maze_size; i++) {
        for (int j = 0; j < maze_size; j++) {
            if (visited[i][j]) {
                visited_cells++;
            }
        }
    }

    return static_cast<double>(visited_cells) / total_cells * 100.0;
}

bool Scout::is_fully_explored() const {
    return fully_explored;
}

const std::vector<std::vector<std::vector<int>>>& Scout::get_knowledge_matrix() const {
    return knowledge_matrix;
}

const std::vector<std::vector<bool>>& Scout::get_visited() const {
    return visited;
}