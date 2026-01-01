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