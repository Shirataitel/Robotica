#include <Krembot/controller/krembot_controller.h>
#include <queue>
#include <vector>
#include <list>
#include <math.h>

using namespace std;

struct MapMsg {
    int **occupancyGrid;
    int **weightedGrid;
    Real resolution;
    CVector2 origin;
    int height, width;
};

struct PosMsg {
    CVector2 pos;
    CDegrees degreeX;
};

struct Direction {
    bool up;
    bool right;
    bool down;
    bool left;
};

struct Degrees {
    CDegrees up_degree = CDegrees(90);
    CDegrees left_degree = CDegrees(180);
    CDegrees down_degree = CDegrees(270);
    CDegrees right_degree = CDegrees(0);
};

class Node {
private:
    int id;
    int x;
    int y;
    int weight;
    bool obstacle;
    bool coarseGrid;
    vector<Node *> neighbors;
public:
    Node(int id, int x, int y, int weight, bool obstacle, bool coarseGrid);

    int getX() const;

    int getY() const;

    int getId() const;

    int getWeight() const;

    bool isObstacle() const;

    bool isCoarseNode() const;

    bool isEqual(Node *otherNode);

    vector<Node *> getNeighbors();

    void addNeighbor(Node *n);

};

class WSTC_controller : public KrembotController {
private:
    Real robotSize = 0.20;
    bool isFirst = true;
public:
    MapMsg mapMsg;
    PosMsg posMsg;

    ParticleObserver Particle;

    ~WSTC_controller() = default;

    void setup();

    void loop();

    void Init(TConfigurationNode &t_node) override {
        KrembotController::Init(t_node);
        if (!krembot.isInitialized()) {
            throw std::runtime_error("krembot.ino.cpp: krembot wasn't initialized in controller");
        }
        Particle.setName(krembot.getName());
    }

    void ControlStep() override {
        if (isFirst) {
            setup();
            isFirst = false;
        }
        loop();
    }

    void init_grid(int **oldGrid, int **oldWeights, int D, int _width, int _height);

    bool got_to_cell(int _col, int _row);

    bool got_to_orientation(CDegrees degree);

    void pos_to_col_row(CVector2 pos, int *pCol, int *pRow);

    void pos_to_col_row_uniform(int *pCol, int *pRow);

    void pos_to_col_row_coarse(int *pCol, int *pRow);

    void save_grid_to_file(string name, int **grid, int _height, int _width);

    void save_grid_to_file_with_robot_location(string name, int **grid,
                                               int _height, int _width,
                                               int robot_col, int robot_row);

    void init_nodes_matrix_coarse(int _width, int _height);

    void init_nodes_matrix_uniform(int _width, int _height);

    void init_neighbors_matrix(int _width, int _height);

    void add_edge(Node *node, int _width, int _height);

    void check_valid_edge(int newX, int newY, Node *node);

    void save_nodes_to_file(string name, Node ***grid, int _height, int _width);

    void save_edges_to_file(string name, int _height, int _width);

    void prim(int numOfNodes);

    bool isExistEdge(Node *node1, Node *node2);

    void init_directions_matrix(int _width, int _height);

    void update_directions_matrix(Node *n1, Node *n2);

    vector<Node *> get_relevant_neighbors(Node *node,  Node *prev);

    void init_path();

    vector<Node *> get_unBlackNodes(vector<Node *> nodes, vector<Node *> blackNodes);

    void free_memory();

    CDegrees calculateWantedDegree(Node *current_cell, Node *next_cell);

    void save_tree_to_file(string name, int **grid,Direction** dir , int _height, int _width);

};


REGISTER_CONTROLLER(WSTC_controller, "WSTC_controller")