#include <iostream>
#include <vector>
#include <queue>
#include <string>
#include <algorithm>
#include <cstdlib>
struct Point {
    int x, y;
};
struct Node {
    Point p;
    char dir;
    int parent;
    std::string command;
};
const int SIZE = 5;
Point robot = {0, 0};
Point goal = {4, 4};
std::vector<Point> obstacles = {
    {2, 2},
    {4, 1},
    {1, 3}
};
std::vector<Point> path;
std::vector<std::string> commandHistory;
char direction = 'N';
int movementCount = 0;
int obstacleCollisions = 0;
int boundaryCollisions = 0;
std::vector<Point> bfs(Point start, Point target);
bool samePoint(Point a, Point b) {
    return a.x == b.x && a.y == b.y;
}
bool inside(Point p) {
    return p.x >= 0 && p.x < SIZE &&
           p.y >= 0 && p.y < SIZE;
}
bool obstacle(Point p) {
    for (const Point& o : obstacles) {
        if (samePoint(p, o))
            return true;
    }
    return false;
}
char rightDir(char d) {
    if (d == 'N') return 'E';
    if (d == 'E') return 'S';
    if (d == 'S') return 'W';
    return 'N';
}
char leftDir(char d) {
    if (d == 'N') return 'W';
    if (d == 'W') return 'S';
    if (d == 'S') return 'E';
    return 'N';
}
char opposite(char d) {
    if (d == 'N') return 'S';
    if (d == 'S') return 'N';
    if (d == 'E') return 'W';
    return 'E';
}
Point nextPoint(Point p, char d) {
    if (d == 'N') p.y++;
    else if (d == 'S') p.y--;
    else if (d == 'E') p.x++;
    else p.x--;
    return p;
}
int dirIndex(char d) {
    if (d == 'N') return 0;
    if (d == 'E') return 1;
    if (d == 'S') return 2;
    return 3;
}
void showGrid() {
    std::cout << "\n--- GRID ---\n";
    for (int y = SIZE - 1; y >= 0; y--) {
        for (int x = 0; x < SIZE; x++) {
            Point p = {x, y};
            if (samePoint(p, robot))
                std::cout << "R ";
            else if (samePoint(p, goal))
                std::cout << "G ";
            else if (obstacle(p))
                std::cout << "X ";
            else
                std::cout << ". ";
        }
        std::cout << '\n';
    }
    std::cout << "--------------------\n";
}
std::vector<Point> bfs(Point start, Point target) {
    bool visited[SIZE][SIZE] = {};
    Point parent[SIZE][SIZE];
    std::queue<Point> q;
    q.push(start);
    visited[start.y][start.x] = true;
    int dx[] = {1, -1, 0, 0};
    int dy[] = {0, 0, 1, -1};
    while (!q.empty()) {
        Point current = q.front();
        q.pop();
        if (samePoint(current, target))
            break;
        for (int i = 0; i < 4; i++) {
            Point next = {
                current.x + dx[i],
                current.y + dy[i]
            };
            if (!inside(next) ||
                obstacle(next) ||
                visited[next.y][next.x])
                continue;
            visited[next.y][next.x] = true;
            parent[next.y][next.x] = current;
            q.push(next);
        }
    }
    if (!visited[target.y][target.x])
        return {};
    std::vector<Point> result;
    Point current = target;
    while (!samePoint(current, start)) {
        result.push_back(current);
        current = parent[current.y][current.x];
    }
    result.push_back(start);
    std::reverse(result.begin(), result.end());
    return result;
}
int distanceToGoal() {
    std::vector<Point> result = bfs(robot, goal);
    if (result.empty())
        return -1;
    return static_cast<int>(result.size()) - 1;
}
std::string sensorResult(Point p) {
    if (!inside(p))
        return "BOUNDARY";
    if (obstacle(p)) {
        return "OBSTACLE at (" +
               std::to_string(p.x) + "," +
               std::to_string(p.y) + ")";
    }
    return "CLEAR";
}
void sensorScan() {
    Point front = nextPoint(robot, direction);
    Point back = nextPoint(robot, opposite(direction));
    Point left = nextPoint(robot, leftDir(direction));
    Point right = nextPoint(robot, rightDir(direction));
    std::cout << "\n--- SENSOR SCAN ---\n";
    std::cout << "Front : " << sensorResult(front) << '\n';
    std::cout << "Back  : " << sensorResult(back) << '\n';
    std::cout << "Left  : " << sensorResult(left) << '\n';
    std::cout << "Right : " << sensorResult(right) << '\n';
    std::cout << "-------------------\n";
}
void showStatus() {
    std::cout << "\n--- ROBOT STATUS ---\n";
    std::cout << "Robot Position: ("
              << robot.x << "," << robot.y << ")\n";
    std::cout << "Direction: " << direction << '\n';
    std::cout << "Goal: ("
              << goal.x << "," << goal.y << ")\n";
    int distance = distanceToGoal();
    if (distance == -1)
        std::cout << "Distance to Goal: No path\n";
    else
        std::cout << "Distance to Goal: "
                  << distance << '\n';
    std::cout << "Movement Count: "
              << movementCount << '\n';
    std::cout << "--------------------\n";
}
void showShortestPath() {
    std::vector<Point> result = bfs(robot, goal);
    std::cout << "\n--- BFS SHORTEST PATH ---\n";
    if (result.empty()) {
        std::cout << "No path available!\n";
    } else {
        for (size_t i = 0; i < result.size(); i++) {
            std::cout << "("
                      << result[i].x << ","
                      << result[i].y << ")";
            if (i + 1 < result.size())
                std::cout << " -> ";
        }
        std::cout << "\nSteps: "
                  << result.size() - 1 << '\n';
    }
    std::cout << "-------------------------\n";
}
std::vector<std::string> minimumCommandPath() {
    bool visited[SIZE][SIZE][4] = {};
    std::vector<Node> nodes;
    std::queue<int> q;
    nodes.push_back({robot, direction, -1, ""});
    visited[robot.y][robot.x][dirIndex(direction)] = true;
    q.push(0);
    int goalNode = -1;
    while (!q.empty()) {
        int index = q.front();
        q.pop();
        Node current = nodes[index];
        if (samePoint(current.p, goal)) {
            goalNode = index;
            break;
        }
        Point forward = nextPoint(current.p, current.dir);
        int d = dirIndex(current.dir);
        if (inside(forward) &&
            !obstacle(forward) &&
            !visited[forward.y][forward.x][d]) {
            visited[forward.y][forward.x][d] = true;
            nodes.push_back({
                forward,
                current.dir,
                index,
                "forward"
            });
            q.push(static_cast<int>(nodes.size() - 1));
        }
        char right = rightDir(current.dir);
        int rd = dirIndex(right);
        if (!visited[current.p.y][current.p.x][rd]) {
            visited[current.p.y][current.p.x][rd] = true;
            nodes.push_back({
                current.p,
                right,
                index,
                "right"
            });
            q.push(static_cast<int>(nodes.size() - 1));
        }
        char left = leftDir(current.dir);
        int ld = dirIndex(left);
        if (!visited[current.p.y][current.p.x][ld]) {
            visited[current.p.y][current.p.x][ld] = true;
            nodes.push_back({
                current.p,
                left,
                index,
                "left"
            });
            q.push(static_cast<int>(nodes.size() - 1));
        }
        Point backward = nextPoint(
            current.p,
            opposite(current.dir)
        );
        int bd = dirIndex(current.dir);
        if (inside(backward) &&
            !obstacle(backward) &&
            !visited[backward.y][backward.x][bd]) {
            visited[backward.y][backward.x][bd] = true;
            nodes.push_back({
                backward,
                current.dir,
                index,
                "backward"
            });
            q.push(static_cast<int>(nodes.size() - 1));
        }
    }
    if (goalNode == -1)
        return {};
    std::vector<std::string> commands;
    while (nodes[goalNode].parent != -1) {
        commands.push_back(nodes[goalNode].command);
        goalNode = nodes[goalNode].parent;
    }
    std::reverse(commands.begin(), commands.end());
    return commands;
}
void showMovementToGoal() {
    std::vector<std::string> commands = minimumCommandPath();
    std::cout << "\n--- SUGGESTED MOVEMENT ---\n";
    if (commands.empty()) {
        if (samePoint(robot, goal))
            std::cout << "Robot is already at the goal!\n";
        else
            std::cout << "No path available!\n";
        std::cout << "------------------------\n";
        return;
    }
    for (size_t i = 0; i < commands.size(); i++) {
        std::cout << commands[i];
        if (i + 1 < commands.size())
            std::cout << " -> ";
    }
    std::cout << "\nTotal Commands: "
              << commands.size() << '\n';
    std::cout << "------------------------\n";
}
bool moveForward() {
    Point next = nextPoint(robot, direction);
    if (!inside(next)) {
        std::cout << "Boundary reached!\n";
        boundaryCollisions++;
        path.push_back({-1, -1});
        return false;
    }
    if (obstacle(next)) {
        std::cout << "Obstacle detected at ("
                  << next.x << ","
                  << next.y << ")!\n";
        obstacleCollisions++;
        path.push_back({-2, -2});
        return false;
    }
    robot = next;
    movementCount++;
    path.push_back(robot);
    std::cout << "Robot moved to ("
              << robot.x << ","
              << robot.y << ")\n";
    return true;
}
bool moveBackward() {
    Point next = nextPoint(
        robot,
        opposite(direction)
    );
    if (!inside(next)) {
        std::cout << "Boundary reached!\n";
        boundaryCollisions++;
        path.push_back({-1, -1});
        return false;
    }
    if (obstacle(next)) {
        std::cout << "Obstacle detected at ("
                  << next.x << ","
                  << next.y << ")!\n";
        obstacleCollisions++;
        path.push_back({-2, -2});
        return false;
    }
    robot = next;
    movementCount++;
    path.push_back(robot);
    std::cout << "Robot moved backward to ("
              << robot.x << ","
              << robot.y << ")\n";
    return true;
}
void turnRightCommand() {
    direction = rightDir(direction);
    std::cout << "Robot turned right.\n";
    std::cout << "Direction: " << direction << '\n';
}
void turnLeftCommand() {
    direction = leftDir(direction);
    std::cout << "Robot turned left.\n";
    std::cout << "Direction: " << direction << '\n';
}
void automaticNavigation() {
    std::cout << "\n--- AUTOMATIC NAVIGATION ---\n";
    while (!samePoint(robot, goal)) {
        std::vector<Point> result = bfs(robot, goal);
        if (result.size() < 2) {
            std::cout << "No path available!\n";
            return;
        }
        Point next = result[1];
        char required = direction;
        if (next.x > robot.x)
            required = 'E';
        else if (next.x < robot.x)
            required = 'W';
        else if (next.y > robot.y)
            required = 'N';
        else
            required = 'S';
        while (direction != required) {
            if (leftDir(direction) == required)
                turnLeftCommand();
            else
                turnRightCommand();
        }
        sensorScan();
        moveForward();
        showGrid();
        showStatus();
    }
    std::cout << "\n*** GOAL REACHED! ***\n";
}
void showHelp() {
    std::cout
        << "\n========== COMMANDS ==========\n"
        << "forward   -> Move forward\n"
        << "backward  -> Move backward\n"
        << "left      -> Turn left\n"
        << "right     -> Turn right\n"
        << "reset     -> Reset robot to (0,0), N\n"
        << "help      -> Show commands\n"
        << "path      -> Show BFS shortest path\n"
        << "auto      -> Automatic navigation\n"
        << "stop      -> Stop simulation\n"
        << "==============================\n";
}
void resetRobot() {
    robot = {0, 0};
    direction = 'N';
    movementCount = 0;
    obstacleCollisions = 0;
    boundaryCollisions = 0;
    path.clear();
    commandHistory.clear();
    path.push_back(robot);
    std::cout << "\nRobot reset successfully!\n";
    showGrid();
}
void showFinalPath() {
    std::cout << "\n--- FINAL PATH ---\n";
    for (size_t i = 0; i < path.size(); i++) {
        if (path[i].x == -1 && path[i].y == -1)
            std::cout << "[B]";
        else if (path[i].x == -2 && path[i].y == -2)
            std::cout << "[X]";
        else
            std::cout << "("
                      << path[i].x << ","
                      << path[i].y << ")";
        if (i + 1 < path.size())
            std::cout << " -> ";
    }
    std::cout << '\n';
}
void finalReport() {
    std::cout << "\n========= FINAL REPORT =========\n";
    std::cout << "Final Position: ("
              << robot.x << ","
              << robot.y << ")\n";
    std::cout << "Final Direction: "
              << direction << '\n';
    std::cout << "Goal: ("
              << goal.x << ","
              << goal.y << ")\n";
    std::cout << "Goal Status: "
              << (samePoint(robot, goal)
                    ? "REACHED"
                    : "NOT REACHED")
              << '\n';
    int distance = distanceToGoal();
    if (distance == -1)
        std::cout << "Distance to Goal: No path\n";
    else
        std::cout << "Distance to Goal: " << distance << '\n';
    std::cout << "\n--- COLLISIONS ---\n";
    std::cout << "Obstacle Collisions: "
              << obstacleCollisions << '\n';
    std::cout << "Boundary Collisions: "
              << boundaryCollisions << '\n';
    std::cout << "Movement Count: "
              << movementCount << '\n';
    std::cout << "\n--- COMMAND HISTORY ---\n";
    if (commandHistory.empty()) {
        std::cout << "No commands recorded.\n";
    } else {
        for (const std::string& command : commandHistory)
            std::cout << command << " ";
        std::cout << '\n';
    }
    showFinalPath();
    std::cout << "\nProgram stopped.\n";
    std::cout << "=================================\n";
}
int main() {
    path.push_back(robot);
    std::cout
        << "====================================\n"
        << "     ROBOT MOVEMENT SIMULATOR\n"
        << "====================================\n";
    showGrid();
    showHelp();
    std::string command;
    while (true) {
        std::cout << "\nEnter command: ";
        std::cin >> command;
        if (command == "forward") {
            commandHistory.push_back(command);
            moveForward();
            showGrid();
            showStatus();
            sensorScan();
            if (samePoint(robot, goal)) {
                std::cout << "\n*** GOAL REACHED! ***\n";
                showGrid();
                finalReport();
                break;
            }
        }
        else if (command == "backward") {
            commandHistory.push_back(command);
            moveBackward();
            showGrid();
            showStatus();
            sensorScan();
            if (samePoint(robot, goal)) {
                std::cout << "\n*** GOAL REACHED! ***\n";
                showGrid();
                finalReport();
                break;
            }
        }
        else if (command == "right") {
            commandHistory.push_back(command);
            turnRightCommand();
            showGrid();
            showStatus();
            sensorScan();
        }
        else if (command == "left") {
            commandHistory.push_back(command);
            turnLeftCommand();
            showGrid();
            showStatus();
            sensorScan();
        }
        else if (command == "reset") {
            resetRobot();
        }
        else if (command == "help") {
            showHelp();
        }
        else if (command == "path") {
            showShortestPath();
        }
        else if (command == "auto") {
            commandHistory.push_back(command);
            automaticNavigation();
            if (samePoint(robot, goal)) {
                finalReport();
                break;
            }
        }
        else if (command == "stop") {
            commandHistory.push_back(command);
            if (samePoint(robot, goal)) {
                std::cout << "\n*** GOAL REACHED! ***\n";
            } else {
                std::cout << "\n*** GOAL NOT REACHED! ***\n";
                showMovementToGoal();
            }
            finalReport();
            break;
        }
        else {
            std::cout << "Invalid command!\n";
            std::cout
                << "Type 'help' to see available commands.\n";
        }
    }
    return 0;
}
