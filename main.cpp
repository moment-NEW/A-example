
#include <vector>
#include <cmath>

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

// 内部函数
float heuristic(PointNode& a, PointNode& b) {
    // 使用曼哈顿距离作为启发式函数
    return abs(a.x - b.x) + abs(a.y - b.y);
}





