#ifndef MAZE_SOLVER
#define MAZE_SOLVER

#include <iostream>
#include <vector>
#include <queue>
#include <algorithm>
#include <tuple>

using namespace std;

#define INF 1e9

// Directions: Right, Down, Left, Up
const int dx[4] = {1, 0, -1, 0};
const int dy[4] = {0, 1, 0, -1};

struct Node {
    int x, y, cost, dir;
    bool operator>(const Node& other) const {
        return cost > other.cost;
    }
};

// Function to get the shortest path with direction-based weights
vector<pair<int, int>> dijkstra(vector<vector<int>>& maze, pair<int, int> start, pair<int, int> goal) {
    int rows = maze.size(), cols = maze[0].size();
    vector<vector<vector<int>>> dist(rows, vector<vector<int>>(cols, vector<int>(4, INF)));
    vector<vector<vector<pair<int, int>>>> prev(rows, vector<vector<pair<int, int>>>(cols, vector<pair<int, int>>(4, {-1, -1})));
    priority_queue<Node, vector<Node>, greater<Node>> pq;

    // Initialize for all possible starting directions
    for (int d = 0; d < 4; d++) {
        pq.push({start.first, start.second, 0, d});
        dist[start.first][start.second][d] = 0;
    }

    while (!pq.empty()) {
        Node current = pq.top();
        pq.pop();

        int x = current.x, y = current.y, cost = current.cost, dir = current.dir;
        
        // Stop if we reach the goal
        if (x == goal.first && y == goal.second) break;

        if (cost > dist[x][y][dir]) continue;

        // Try all 4 directions
        for (int i = 0; i < 4; i++) {
            int nx = x + dx[i], ny = y + dy[i];

            // Check if within maze bounds and not a wall
            if (nx >= 0 && ny >= 0 && nx < rows && ny < cols && maze[nx][ny] != 1) {
                int newCost = cost + (i == dir ? 1 : 3);  // 1 for straight, 3 for turns

                if (newCost < dist[nx][ny][i]) {
                    dist[nx][ny][i] = newCost;
                    prev[nx][ny][i] = {x, y};
                    pq.push({nx, ny, newCost, i});
                }
            }
        }
    }

    // Find the best path from any direction at the goal
    int minDist = INF, bestDir = -1;
    for (int d = 0; d < 4; d++) {
        if (dist[goal.first][goal.second][d] < minDist) {
            minDist = dist[goal.first][goal.second][d];
            bestDir = d;
        }
    }

    // Reconstruct path
    vector<pair<int, int>> path;
    pair<int, int> step = goal;
    int dir = bestDir;
    
    while (step != start) {
        if (prev[step.first][step.second][dir] == make_pair(-1, -1)) return {}; // No path found
        path.push_back(step);
        step = prev[step.first][step.second][dir];
    }
    path.push_back(start);
    reverse(path.begin(), path.end());
    return path;
}

#define UNKNOWN -1
#define WALL 1
#define OPEN 0

// Simulated line sensor function (Replace with actual sensor input)
bool lineSensorTriggered() {
    return true;  // Simulating a successful line crossing (replace with real data)
}

// Simulated function to check if there's a wall (Replace with actual sensor input)
bool isWall(int x, int y, vector<vector<int>>& maze) {
    if (x < 0 || y < 0 || x >= maze.size() || y >= maze[0].size()) return true;
    return maze[x][y] == WALL;
}

// Function to move the robot and discover the maze using the line sensor
vector<vector<int>> scoutRun(pair<int, int> start, int rows, int cols, vector<vector<int>>& realMaze) {
    vector<vector<int>> discoveredMaze(rows, vector<int>(cols, UNKNOWN));  // Unknown map initially
    queue<Node> q;
    
    q.push({start.first, start.second, 0});
    discoveredMaze[start.first][start.second] = OPEN;  // Start is open

    while (!q.empty()) {
        Node current = q.front();
        q.pop();

        for (int i = 0; i < 4; i++) {
            int nx = current.x + dx[i], ny = current.y + dy[i];

            if (nx >= 0 && ny >= 0 && nx < rows && ny < cols && discoveredMaze[nx][ny] == UNKNOWN) {
                if (isWall(nx, ny, realMaze)) {
                    discoveredMaze[nx][ny] = WALL;  // Mark wall
                } else {
                    // Only move if the line sensor confirms crossing
                    if (lineSensorTriggered()) {
                        discoveredMaze[nx][ny] = OPEN;  // Mark open path
                        q.push({nx, ny, current.cost + 1});
                    }
                }
            }
        }
    }

    return discoveredMaze;  // Return explored maze
}

// Function to print the discovered maze
void printMaze(vector<vector<int>>& maze) {
    for (auto& row : maze) {
        for (auto& cell : row) {
            if (cell == UNKNOWN) cout << " ? ";
            else if (cell == WALL) cout << " X ";
            else cout << " . ";
        }
        cout << endl;
    }
}

int main() {
    // Simulated real maze (robot doesn't know this initially)
    vector<vector<int>> realMaze = {
        {0, 0, 0, 0, 0},
        {1, 0, 1, 0, 1},
        {0, 0, 0, 0, 0},
        {1, 0, 1, 0, 1},
        {0, 0, 0, 0, 0}
    };

    int rows = realMaze.size(), cols = realMaze[0].size();
    pair<int, int> start = {0, 0};

    vector<vector<int>> discoveredMaze = scoutRun(start, rows, cols, realMaze);

    cout << "Discovered Maze:" << endl;
    printMaze(discoveredMaze);

    return 0;
}


#endif // MAZE_SOLVER