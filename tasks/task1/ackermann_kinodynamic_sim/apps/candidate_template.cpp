/**
 * ============================================================================
 * INTER IIT TECH MEET - CANDIDATE STARTER CLIENT TEMPLATE
 * ============================================================================
 * Scenario: Autonomous Ackermann Vehicle Path Planning & Control
 * Objective: Connect to the TCP simulator, query scenario metadata, solve 
 *            path planning/navigation, and stream control commands (v, delta).
 * ============================================================================
 */

#include <iostream>
#include <string>
#include <sstream>
#include <vector>
#include <cmath>
#include <chrono>
#include <thread>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>
#include <algorithm>
#include <queue>
#include <unordered_map>
#include <tuple>
struct VehicleParams {
    double length = 4.0;
    double width = 1.8;
    double wheelbase = 2.5;
    double max_steer = 0.60; // rad
    double max_speed = 2.5;  // m/s
    double min_speed = -1.5; // m/s
};

struct Pose2D {
    double x = 0.0;
    double y = 0.0;
    double yaw = 0.0;
    double v = 0.0;
    double delta = 0.0;
};

enum Direction { FORWARD = 0, REVERSE = 1 };

struct StateKey {
    int x_bin;
    int y_bin;
    int yaw_bin;
    Direction dir;

    bool operator==(const StateKey& other) const {
        return x_bin == other.x_bin && y_bin == other.y_bin && yaw_bin == other.yaw_bin && dir == other.dir;
    }
};

struct StateKeyHash {
    std::size_t operator()(const StateKey& k) const {
        std::size_t h1 = std::hash<int>()(k.x_bin);
        std::size_t h2 = std::hash<int>()(k.y_bin);
        std::size_t h3 = std::hash<int>()(k.yaw_bin);
        std::size_t h4 = std::hash<int>()(k.dir);
        return h1 ^ (h2 << 1) ^ (h3 << 2) ^ (h4 << 3);
    }
};

struct AStarNode {
    Pose2D state;
    double g_cost;
    double h_cost;
    int parent_index;
    double v_used;
    double delta_used;
    Direction dir;
};

class CandidateSolver {
public:
    CandidateSolver() {}

    /**
     * @param start Initial vehicle pose (x, y, yaw)
     * @param goal Target vehicle pose (x, y, yaw)
     * @param grid 1D vector representing binary occupancy grid (0=Free, 1=Obstacle)
     * @param cols Grid columns count
     * @param rows Grid rows count
     * @param res Grid cell resolution in meters (0.2m)
     * @param params Vehicle physical dimensions and steering limits
     * @return std::vector<Pose2D> Planned trajectory
     */
    std::vector<Pose2D> solve(const Pose2D& start, const Pose2D& goal, 
                              const std::vector<uint8_t>& grid, int cols, int rows, double res,
                              double orig_x, double orig_y,
                              const VehicleParams& params) {
        std::cout << "[Candidate Template] Running solver..." << std::endl;
        std::vector<Pose2D> path;

        // 1. Convert a world (x,y) into grid (gx,gy)
        auto worldToGrid = [&](double wx, double wy, int& gx, int& gy) {
            gx = static_cast<int>(std::floor((wx - orig_x) / res));
            gy = static_cast<int>(std::floor((wy - orig_y) / res));
        };

        // 2. Check whether that grid cell is an obstacle
        auto isObstacle = [&](double wx, double wy) -> bool {
            int gx, gy;
            worldToGrid(wx, wy, gx, gy);
            if (gx < 0 || gx >= cols || gy < 0 || gy >= rows) return true; // Treat outside map as obstacle
            return grid[gy * cols + gx] != 0;
        };

        // 3. Vehicle Kinematic Propagation
        auto stepVehicle = [&](const Pose2D& state, double v, double delta, double dt) -> Pose2D {
            Pose2D next = state;
            // Respect vehicle speed and steering limits
            double cmd_v = std::max(params.min_speed, std::min(params.max_speed, v));
            double cmd_delta = std::max(-params.max_steer, std::min(params.max_steer, delta));
            
            // Apply Ackermann kinematics
            next.x += cmd_v * std::cos(state.yaw) * dt;
            next.y += cmd_v * std::sin(state.yaw) * dt;
            next.yaw += (cmd_v / params.wheelbase) * std::tan(cmd_delta) * dt;
            
            // Normalize yaw to [-pi, pi]
            next.yaw = std::atan2(std::sin(next.yaw), std::cos(next.yaw));
            
            next.v = cmd_v;
            next.delta = cmd_delta;
            return next;
        };

        // 4. Collision checking for a full state
        auto isStateCollision = [&](const Pose2D& state) -> bool {
            double x_rear = -0.8;
            double x_front = params.wheelbase + 0.7;
            double y_left = params.width / 2.0;
            double y_right = -params.width / 2.0;

            std::vector<std::pair<double, double>> local_points;
            
            // Vehicle center
            local_points.push_back({(x_front + x_rear) / 2.0, 0.0});
            
            double spacing = res / 2.0;

            // Top and bottom edges
            for (double x = x_rear; x <= x_front; x += spacing) {
                local_points.push_back({x, y_left});
                local_points.push_back({x, y_right});
            }
            // Ensure exact corners are checked
            local_points.push_back({x_front, y_left});
            local_points.push_back({x_front, y_right});
            local_points.push_back({x_rear, y_left});
            local_points.push_back({x_rear, y_right});

            // Left and right edges (front and rear bumpers)
            for (double y = y_right; y <= y_left; y += spacing) {
                local_points.push_back({x_front, y});
                local_points.push_back({x_rear, y});
            }

            double cos_yaw = std::cos(state.yaw);
            double sin_yaw = std::sin(state.yaw);

            for (const auto& p : local_points) {
                double xl = p.first;
                double yl = p.second;
                double xw = state.x + xl * cos_yaw - yl * sin_yaw;
                double yw = state.y + xl * sin_yaw + yl * cos_yaw;

                if (isObstacle(xw, yw)) {
                    return true;
                }
            }
            return false;
        };

        // 5. Check if a motion is collision free
        auto isMotionCollisionFree = [&](const Pose2D& state, double v, double delta, double dt, int num_steps) -> bool {
            Pose2D curr = state;
            for (int i = 0; i < num_steps; ++i) {
                curr = stepVehicle(curr, v, delta, dt);
                if (isStateCollision(curr)) {
                    return false;
                }
            }
            return true;
        };

        // --------------------------------------------------------------------
        // HYBRID A* IMPLEMENTATION
        // --------------------------------------------------------------------
        
        auto getYawDiff = [](double yaw1, double yaw2) -> double {
            double diff = yaw1 - yaw2;
            while(diff > M_PI) diff -= 2.0 * M_PI;
            while(diff < -M_PI) diff += 2.0 * M_PI;
            return std::abs(diff);
        };

        auto getKey = [](const Pose2D& state, Direction dir) -> StateKey {
            StateKey k;
            k.x_bin = static_cast<int>(std::floor(state.x / 0.5));
            k.y_bin = static_cast<int>(std::floor(state.y / 0.5));
            double yaw_norm = state.yaw;
            while(yaw_norm < 0) yaw_norm += 2.0 * M_PI;
            while(yaw_norm >= 2.0 * M_PI) yaw_norm -= 2.0 * M_PI;
            k.yaw_bin = static_cast<int>(std::floor(yaw_norm / (15.0 * M_PI / 180.0)));
            k.dir = dir;
            return k;
        };

        auto getHeuristic = [&](const Pose2D& state) -> double {
            double dist = std::hypot(goal.x - state.x, goal.y - state.y);
            double heading_err = getYawDiff(state.yaw, goal.yaw);
            return dist + 0.1 * heading_err;
        };
        
        auto isGoal = [&](const Pose2D& state) -> bool {
            double dist = std::hypot(goal.x - state.x, goal.y - state.y);
            double heading_err = getYawDiff(state.yaw, goal.yaw);
            return (dist < 0.40 && heading_err < 0.30);
        };

        std::vector<AStarNode> all_nodes;
        std::unordered_map<StateKey, double, StateKeyHash> closed_set;
        
        auto cmp = [&](int a, int b) {
            double fa = all_nodes[a].g_cost + all_nodes[a].h_cost;
            double fb = all_nodes[b].g_cost + all_nodes[b].h_cost;
            return fa > fb; // smallest f first
        };
        std::priority_queue<int, std::vector<int>, decltype(cmp)> open_set(cmp);

        // Constants for penalty weights
        const double W_DIST = 1.0;
        const double W_REVERSE = 2.0;
        const double W_SWITCH = 10.0;
        const double W_STEER_CHANGE = 0.5;

        // Initialize Start Node
        AStarNode start_node;
        start_node.state = start;
        start_node.g_cost = 0.0;
        start_node.h_cost = getHeuristic(start);
        start_node.parent_index = -1;
        start_node.v_used = 0.0;
        start_node.delta_used = 0.0;
        start_node.dir = FORWARD;
        
        all_nodes.push_back(start_node);
        open_set.push(0);
        closed_set[getKey(start, FORWARD)] = 0.0;
        
        // Define motion primitives parameters
        double sim_dt = 0.1;
        int sim_steps = 5;
        double steering_vals[5] = {
            -params.max_steer,
            -0.5 * params.max_steer,
            0.0,
            0.5 * params.max_steer,
            params.max_steer
        };
        double speeds[2] = { params.max_speed, params.min_speed };
        Direction dirs[2] = { FORWARD, REVERSE };

        int goal_node_idx = -1;
        int expanded_nodes = 0;
        
        while (!open_set.empty()) {
            int curr_idx = open_set.top();
            open_set.pop();
            const AStarNode& curr_node = all_nodes[curr_idx];
            
            expanded_nodes++;
            if (expanded_nodes % 5000 == 0) {
                std::cout << "[Candidate Template] Expanded " << expanded_nodes << " nodes..." << std::endl;
            }

            if (isGoal(curr_node.state)) {
                goal_node_idx = curr_idx;
                break;
            }
            
            for (int d = 0; d < 2; ++d) {
                double v = speeds[d];
                Direction next_dir = dirs[d];
                
                for (int s = 0; s < 5; ++s) {
                    double delta = steering_vals[s];
                    
                    // Simulate motion primitive
                    Pose2D sim_state = curr_node.state;
                    bool collision = false;
                    double dist_travelled = 0.0;
                    
                    for (int step = 0; step < sim_steps; ++step) {
                        Pose2D next_state = stepVehicle(sim_state, v, delta, sim_dt);
                        if (isStateCollision(next_state)) {
                            collision = true;
                            break;
                        }
                        dist_travelled += std::hypot(next_state.x - sim_state.x, next_state.y - sim_state.y);
                        sim_state = next_state;
                    }
                    
                    if (collision) continue;
                    
                    // Calculate costs
                    double g_new = curr_node.g_cost + (dist_travelled * W_DIST);
                    if (next_dir == REVERSE) g_new += (dist_travelled * W_REVERSE);
                    if (next_dir != curr_node.dir) g_new += W_SWITCH;
                    g_new += std::abs(delta - curr_node.delta_used) * W_STEER_CHANGE;
                    
                    StateKey key = getKey(sim_state, next_dir);
                    
                    auto it = closed_set.find(key);
                    if (it == closed_set.end() || g_new < it->second) {
                        closed_set[key] = g_new;
                        
                        AStarNode next_node;
                        next_node.state = sim_state;
                        next_node.g_cost = g_new;
                        next_node.h_cost = getHeuristic(sim_state);
                        next_node.parent_index = curr_idx;
                        next_node.v_used = v;
                        next_node.delta_used = delta;
                        next_node.dir = next_dir;
                        
                        // Pass along the applied motion controls safely to state 
                        next_node.state.v = v;
                        next_node.state.delta = delta;
                        
                        all_nodes.push_back(next_node);
                        open_set.push(all_nodes.size() - 1);
                    }
                }
            }
        }
        
        if (goal_node_idx != -1) {
            std::cout << "[Candidate Template] Path found! Nodes expanded: " << expanded_nodes << std::endl;
            int curr = goal_node_idx;
            while(curr != -1) {
                path.push_back(all_nodes[curr].state);
                curr = all_nodes[curr].parent_index;
            }
            std::reverse(path.begin(), path.end());
        } else {
            std::cout << "[Candidate Template] Hybrid A* failed to find a path! Expanded: " << expanded_nodes << std::endl;
        }

        return path;
    }
};

int main(int argc, char** argv) {
    std::string ip = "127.0.0.1";
    int port = 8091;

    if (argc > 1) ip = argv[1];
    if (argc > 2) port = std::atoi(argv[2]);

    std::cout << "==========================================================" << std::endl;
    std::cout << "      Inter IIT Tech Meet: Candidate Client Template     " << std::endl;
    std::cout << "==========================================================" << std::endl;

    int sock = socket(AF_INET, SOCK_STREAM, 0);
    sockaddr_in serv_addr{};
    serv_addr.sin_family = AF_INET;
    serv_addr.sin_port = htons(port);
    inet_pton(AF_INET, ip.c_str(), &serv_addr.sin_addr);

    if (connect(sock, reinterpret_cast<sockaddr*>(&serv_addr), sizeof(serv_addr)) < 0) {
        std::cerr << "[Client] Connection failed. Is simulator_node running on port " << port << "?" << std::endl;
        return 1;
    }

    std::cout << "[Client] Connected to simulator server!" << std::endl;

    // Send query
    std::string query = "Q\n";
    write(sock, query.c_str(), query.length());

    char buffer[16384];
    ssize_t bytes = read(sock, buffer, sizeof(buffer) - 1);
    if (bytes <= 0) return 1;
    buffer[bytes] = '\0';

    Pose2D start, goal;
    VehicleParams v_params;
    double map_w, map_h, res, orig_x, orig_y;
    int cols = 0, rows = 0;
    std::vector<uint8_t> grid;

    std::string msg(buffer);
    std::istringstream ss(msg);
    std::string line;

    while (std::getline(ss, line)) {
        if (line.rfind("CONFIG", 0) == 0) {
            std::istringstream line_ss(line.substr(7));
            line_ss >> start.x >> start.y >> start.yaw
                    >> goal.x >> goal.y >> goal.yaw
                    >> v_params.length >> v_params.width >> v_params.wheelbase
                    >> v_params.max_steer >> v_params.max_speed >> v_params.min_speed
                    >> map_w >> map_h >> res >> orig_x >> orig_y
                    >> cols >> rows;
        }
        else if (line.rfind("GRID", 0) == 0) {
            std::istringstream line_ss(line.substr(5));
            size_t count;
            line_ss >> count;
            int val;
            while (line_ss >> val) grid.push_back(val ? 1 : 0);
        }
    }

    std::cout << "[Client] Config loaded. Map: " << cols << "x" << rows << " resolution: " << res << "m" << std::endl;

    CandidateSolver solver;
    auto path = solver.solve(start, goal, grid, cols, rows, res, orig_x, orig_y, v_params);

    size_t target_idx = 0;
    while (true) {
        bytes = read(sock, buffer, sizeof(buffer) - 1);
        if (bytes <= 0) break;
        buffer[bytes] = '\0';

        std::string telem_str(buffer);
        if (telem_str.find("TELEMETRY") != std::string::npos) {
            std::istringstream t_ss(telem_str);
            std::string tag;
            uint64_t step;
            double t_ms, cur_x, cur_y, cur_yaw, cur_v, cur_delta;
            int coll, goal_done;
            t_ss >> tag >> step >> t_ms >> cur_x >> cur_y >> cur_yaw >> cur_v >> cur_delta >> coll >> goal_done;

            if (coll) { std::cout << "[Client] Collision detected!" << std::endl; break; }
            if (goal_done) { std::cout << "[Client] Goal reached!" << std::endl; break; }

            if (target_idx < path.size()) {
                std::ostringstream cmd_ss;
                cmd_ss << "CTRL " << path[target_idx].v << " " << path[target_idx].delta << "\n";
                std::string cmd_str = cmd_ss.str();
                write(sock, cmd_str.c_str(), cmd_str.length());
                target_idx++;
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }

    close(sock);
    return 0;
}
