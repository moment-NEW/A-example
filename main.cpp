#include "main.hpp"



class PointNode {
public:
    int x,y;
    float origin_cost;
    float final_cost;
    float total_cost;
    bool Open_flag=false;//是否在open表中
    bool Close_flag=false;//是否在closed表中
    PointNode* parent;//父节点，用来回溯路径

    PointNode(int x, int y, float origin_cost = 0.0, float final_cost = 0.0, PointNode* parent = nullptr)
        : x(x), y(y), origin_cost(origin_cost), final_cost(final_cost), parent(parent) {
            total_cost = origin_cost + final_cost;
        }
private:
    int value;
    
};



/**
 * @brief 小堆比较器
 */
struct MyCompare {
    bool operator()(PointNode* a, PointNode* b) const {
        return a->total_cost > b->total_cost;
    }
};


// 内部函数
float heuristic(const PointNode& a, const PointNode& b) {
    // 使用曼哈顿距离作为启发式函数
    return abs(a.x - b.x) + abs(a.y - b.y);
}
/**
 * @brief 欧几里得距离函数
 * 
 * 
 */

float euclidean_distance(const PointNode& a, const PointNode& b) {
    return std::sqrt(std::pow(a.x - b.x, 2) + std::pow(a.y - b.y, 2));
}




//变量
priority_queue<PointNode*, vector<PointNode*>, MyCompare> openList;//早知道不用堆了，麻烦死了，不能遍历
vector<PointNode*> closedList;//cpp的动态数组
vector<vector<int>> grid; // 网格地图，0表示可通行，1表示障碍物

A_star_params astar_params={
.start_x=0,
.start_y=0,
.goal_x=0,
.goal_y=0

};

//主函数

/**
 * @brief A*算法主函数
 * 
 */
PointNode A_star_caculate(int origin_x, int origin_y, int target_x, int target_y) {
    PointNode* startNode = new PointNode(origin_x, origin_y);
    PointNode* targetNode = new PointNode(target_x, target_y);

    openList.push(startNode);

    while (!openList.empty()) {
        PointNode* currentNode = openList.top();
        openList.pop();
        closedList.push_back(currentNode);

        // 检查是否到达目标
        if (currentNode->x == targetNode->x && currentNode->y == targetNode->y) {
            return *currentNode; // 找到路径，返回目标节点
        }

        // 生成邻居节点（上下左右四个方向）
        vector<PointNode*> neighbors;
        vector<pair<int, int>> directions = {{0,1}, {1,0}, {0,-1}, {-1,0}};
        for (auto& dir : directions) {
            int newX = currentNode->x + dir.first;
            int newY = currentNode->y + dir.second;

            // 检查边界和障碍物
            if (newX >= 0 && newX < grid.size() && newY >= 0 && newY < grid[0].size() && grid[newX][newY] == 0) {
                PointNode* neighbor = new PointNode(newX, newY);
                neighbors.push_back(neighbor);
            }
        }

        // 处理每个邻居节点
        for (auto& neighbor : neighbors) {
            // 如果邻居在closedList中，跳过
            bool inClosedList = false;
            for (auto& closedNode : closedList) {
                if (neighbor->x == closedNode->x && neighbor->y == closedNode->y) {
                    inClosedList = true;
                    neighbor->Close_flag = true;
                    break;
                }
            }
            if (inClosedList) continue;

            // 计算成本
            float tentative_origin_cost = currentNode->origin_cost + 1.0; // 假设每步代价为1
            float final_cost = heuristic(*neighbor, *targetNode);
            neighbor->origin_cost = tentative_origin_cost;
            neighbor->final_cost = final_cost;
            neighbor->total_cost = tentative_origin_cost + final_cost;
            neighbor->parent = currentNode;

            // 如果邻居不在openList中，加入openList
            if (!neighbor->Open_flag) {
                neighbor->Open_flag = true;
                openList.push(neighbor);
            }
        }
    }
    
    // 没有找到路径
    return PointNode(-1, -1); // 返回无效节点表示失败
}

// 辅助函数：重建路径
vector<pair<int, int>> reconstructPath(PointNode* endNode) {
    vector<pair<int, int>> path;
    PointNode* current = endNode;
    while (current != nullptr) {
        path.push_back({current->x, current->y});
        current = current->parent;
    }
    reverse(path.begin(), path.end());
    return path;
}

// 主函数
#ifndef AS_LIBRARY
int main() {
    // 初始化地图 (10x10的网格)
    grid = {
        {0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
        {0, 1, 1, 0, 0, 0, 1, 0, 0, 0},
        {0, 0, 0, 0, 1, 0, 1, 0, 0, 0},
        {0, 0, 1, 0, 1, 0, 0, 0, 0, 0},
        {0, 0, 1, 0, 0, 0, 1, 1, 0, 0},
        {0, 0, 0, 0, 1, 0, 0, 0, 0, 0},
        {0, 1, 1, 0, 1, 0, 1, 1, 1, 0},
        {0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
        {0, 0, 1, 1, 1, 1, 0, 0, 0, 0},
        {0, 0, 0, 0, 0, 0, 0, 0, 0, 0}
    };

    // 设置起点和终点
    int start_x = 0, start_y = 0;
    int goal_x = 9, goal_y = 9;

    printf("A* 算法路径规划\n");
    printf("起点: (%d, %d)\n", start_x, start_y);
    printf("终点: (%d, %d)\n", goal_x, goal_y);
    printf("\n地图 (0=可通行, 1=障碍物):\n");
    for (int i = 0; i < grid.size(); i++) {
        for (int j = 0; j < grid[i].size(); j++) {
            printf("%d ", grid[i][j]);
        }
        printf("\n");
    }

    // 执行A*算法
    PointNode result = A_star_caculate(start_x, start_y, goal_x, goal_y);

    // 检查是否找到路径
    if (result.x == -1 && result.y == -1) {
        printf("\n未找到路径！\n");
    } else {
        printf("\n找到路径！\n");
        printf("目标位置: (%d, %d)\n", result.x, result.y);
        
        // 重建并打印路径
        vector<pair<int, int>> path = reconstructPath(&result);
        printf("\n路径 (共 %lu 步):\n", path.size());
        for (size_t i = 0; i < path.size(); i++) {
            printf("步骤 %lu: (%d, %d)\n", i, path[i].first, path[i].second);
        }
        
        // 在地图上显示路径
        printf("\n路径地图 (S=起点, E=终点, *=路径, 1=障碍物, 0=空地):\n");
        vector<vector<char>> pathMap(grid.size(), vector<char>(grid[0].size()));
        for (int i = 0; i < grid.size(); i++) {
            for (int j = 0; j < grid[i].size(); j++) {
                if (grid[i][j] == 1) {
                    pathMap[i][j] = '1';
                } else {
                    pathMap[i][j] = '0';
                }
            }
        }
        
        for (auto& p : path) {
            pathMap[p.first][p.second] = '*';
        }
        pathMap[start_x][start_y] = 'S';
        pathMap[goal_x][goal_y] = 'E';
        
        for (int i = 0; i < pathMap.size(); i++) {
            for (int j = 0; j < pathMap[i].size(); j++) {
                printf("%c ", pathMap[i][j]);
            }
            printf("\n");
        }
    }

    // 清理内存（释放动态分配的节点）
    while (!openList.empty()) {
        delete openList.top();
        openList.pop();
    }
    for (auto node : closedList) {
        delete node;
    }

    return 0;
}
#endif