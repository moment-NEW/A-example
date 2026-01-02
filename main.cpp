
#include <vector>
#include <queue>
#include <cmath>

using namespace std;//暂时这么做

class PointNode {
public:
    int x,y;
    float origin_cost;
    float final_cost;
    float total_cost;
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
priority_queue<PointNode*, vector<PointNode*>, MyCompare> openList;
vector<PointNode*> closedList;//cpp的动态数组
vector<vector<int>> grid; // 网格地图，0表示可通行，1表示障碍物



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
            bool inOpenList = false;    
            // 这里需要遍历openList来检查邻居是否已经存在，但priority_queue不支持遍历，需改用其他数据结构或辅助结构


        }
    }
}