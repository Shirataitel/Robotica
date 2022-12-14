#include "krembot.ino.h"

#define INF 9999999

int col, row, colU, rowU, colC, rowC;
int **occupancyGrid;
int **uniformGrid;
int **coarseGrid;
int **weights;
int **weightsUniform;
int **weightsCoarse;
Node ***nodesMatrix;
int **neighborsMatrix;
Real resolution;
CVector2 origin;
int height, width;
CVector2 pos;
CDegrees degreeX;
int robotGridSize;
vector<pair<int, int>> mst;
//bool first_time = true;


enum State {
    move,
    turn
} state = turn;

void WSTC_controller::setup() {
    krembot.setup();
    krembot.Led.write(0, 255, 0);

    occupancyGrid = mapMsg.occupancyGrid;
    weights = mapMsg.weightedGrid;
    resolution = mapMsg.resolution;
    origin = mapMsg.origin;
    height = mapMsg.height;
    width = mapMsg.width;
    robotGridSize = (int) (robotSize / resolution);

    // occupancyGrid
    save_grid_to_file("/home/oriya/krembot_sim/krembot_ws/WSTC/grid.txt", occupancyGrid, height, width);
    pos = posMsg.pos;
    degreeX = posMsg.degreeX;
    pos_to_col_row(pos, &col, &row);
    save_grid_to_file_with_robot_location("/home/oriya/krembot_sim/krembot_ws/files/grid-with-robot-loc.txt",
                                          occupancyGrid, height, width, col, row);


    // uniformGrid
    init_grid(occupancyGrid, weights, robotGridSize, width, height);
    int h = height / robotGridSize;
    int w = width / robotGridSize;
    save_grid_to_file("/home/oriya/krembot_sim/krembot_ws/files/uniform-grid.txt", uniformGrid, h, w);
    save_grid_to_file("/home/oriya/krembot_sim/krembot_ws/files/uniform-weights.txt", weightsUniform, h, w);
    pos_to_col_row_uniform(&col, &row);
    save_grid_to_file_with_robot_location("/home/oriya/krembot_sim/krembot_ws/files/uniform-with-robot-loc.txt",
                                          uniformGrid, h, w, colU, rowU);


    // coarseGrid
    init_grid(uniformGrid, weightsUniform, 2, w, h);
    save_grid_to_file("/home/oriya/krembot_sim/krembot_ws/files/coarse-grid.txt", coarseGrid, h / 2, w / 2);
    save_grid_to_file("/home/oriya/krembot_sim/krembot_ws/files/coarse-weights.txt", weightsCoarse, h / 2, w / 2);
    pos_to_col_row_coarse(&colU, &rowU);
    save_grid_to_file_with_robot_location("/home/oriya/krembot_sim/krembot_ws/files/coarse-with-robot-loc.txt",
                                          coarseGrid, h / 2, w / 2, colC, rowC);


    // nodesMatrix
    init_nodes_matrix(w / 2, h / 2);
    save_nodes_to_file("/home/oriya/krembot_sim/krembot_ws/files/nodes.txt", h / 2, w / 2);

    // neighborsMatrix
    int numOfNodes = (w / 2) * (h / 2);
    init_neighbors_matrix(w / 2, h / 2);
    save_edges_to_file("/home/oriya/krembot_sim/krembot_ws/files/neighbors.txt", numOfNodes, numOfNodes);

    prim(numOfNodes);

    free_memory();
}

void WSTC_controller::prim(int numOfNodes) {
    Node *startNode = nodesMatrix[rowC][colC];
    int startNodeId = startNode->getId();
    int numOfEdges = 0;
    bool *selected = new bool[numOfNodes];
    for (int i = 0; i < numOfNodes; i++) {
        selected[i] = false;
    }
    selected[startNodeId] = true;
    int x;
    int y;
    while (numOfEdges < numOfNodes - 1) {
        int min = INF;
        x = 0;
        y = 0;
        for (int i = 0; i < numOfNodes; i++) {
            if (selected[i]) {
                for (int j = 0; j < numOfNodes; j++) {
                    if (!selected[j] && (neighborsMatrix[i][j] != -1)) {
                        if (min > neighborsMatrix[i][j]) {
                            min = neighborsMatrix[i][j];
                            x = i;
                            y = j;
                        }
                    }
                }
            }
        }
        if(neighborsMatrix[x][y]!= -1){
            LOG << x << " - " << y << " :  " << neighborsMatrix[x][y]<< endl;
            mst.push_back(make_pair(x, y));
        }
        selected[y] = true;
        numOfEdges++;
    }
    LOG<<mst.size()<<endl;
    delete[] selected;
}

void WSTC_controller::free_memory() {
    for (int i = 0; i < width / robotGridSize; i++) {
        delete[] uniformGrid[i];
        delete[] weightsUniform[i];
    }
    delete[] uniformGrid;
    delete[] weightsUniform;

    for (int i = 0; i < width / robotGridSize / 2; i++) {
        delete[] coarseGrid[i];
        delete[] weightsCoarse[i];
    }
    delete[] coarseGrid;
    delete[] weightsCoarse;

    for (int i = 0; i < width / robotGridSize / 2; i++) {
        for (int j = 0; j < height / robotGridSize / 2; j++) {
            delete nodesMatrix[i][j];
        }
        delete[] nodesMatrix[i];
    }
    delete[] nodesMatrix;

    int numOfNodes = (width / robotGridSize / 2) * (height / robotGridSize / 2);
    for (int i = 0; i < numOfNodes; i++) {
        delete[] neighborsMatrix[i];
    }
    delete[] neighborsMatrix;
}

void WSTC_controller::loop() {
    krembot.loop();

    pos = posMsg.pos;
    degreeX = posMsg.degreeX;

    switch (state) {
        case State::move: {
            if (!got_to_cell(col + 3, row)) {
                krembot.Base.drive(100, 0);
            } else {
                krembot.Base.stop();
            }
            break;
        }
        case State::turn: {
            if (!got_to_orientation(CDegrees(0))) {
                krembot.Base.drive(0, 20);
            } else {
                krembot.Base.stop();
                state = State::move;
            }
            break;
        }
    }
}

void WSTC_controller::init_nodes_matrix(int _width, int _height) {
    nodesMatrix = new Node **[_width];
    for (int i = 0; i < _width; i++) {
        nodesMatrix[i] = new Node *[_height];
    }
    int id = 0;
    int weight = 0;
    bool isObstacle = false;
    for (int i = 0; i < _height; ++i) {
        for (int j = 0; j < _width; j++) {
            if (coarseGrid[i][j] == 1) {
                isObstacle = true;
            } else {
                isObstacle = false;
            }
            weight = weightsCoarse[i][j];
            nodesMatrix[i][j] = new Node(id, i, j, weight, isObstacle);
            id++;
        }
    }
}

void WSTC_controller::init_neighbors_matrix(int _width, int _height) {
    int numOfNodes = _width * _height;
    neighborsMatrix = new int *[numOfNodes];
    for (int i = 0; i < numOfNodes; i++) {
        neighborsMatrix[i] = new int[numOfNodes];
    }

    for (int i = 0; i < numOfNodes; ++i) {
        for (int j = 0; j < numOfNodes; j++) {
            neighborsMatrix[i][j] = -1;
        }
    }

    Node *node;
    for (int i = 0; i < _height; ++i) {
        for (int j = 0; j < _width; j++) {
            node = nodesMatrix[i][j];
            if (!node->isObstacle()) {
                add_edge(node, _width, _height);
            }
        }
    }
}

void WSTC_controller::add_edge(Node *node, int _width, int _height) {
    int id = node->getId();
    int x = node->getX();
    int y = node->getY();
    int weight = node->getWeight();
    int newX, newY, maxWeight;
    Node *neighbor;

    // up
    newX = x + 1;
    newY = y;
    if (newX < _height) {
        check_valid_edge(newX, newY, node);
    }


    // down
    newX = x - 1;
    newY = y;
    if (newX >= 0) {
        check_valid_edge(newX, newY, node);
    }

    // right
    newX = x;
    newY = y + 1;
    if (newY < _width) {
        check_valid_edge(newX, newY, node);
    }

    // left
    newX = x;
    newY = y - 1;
    if (newY >= 0) {
        check_valid_edge(newX, newY, node);
    }
}

void WSTC_controller::check_valid_edge(int newX, int newY, Node *node) {
    Node *neighbor = nodesMatrix[newX][newY];
    int maxWeight = 0;
    if (!neighbor->isObstacle()) {
        maxWeight = max(node->getWeight(), neighbor->getWeight());
        neighborsMatrix[node->getId()][neighbor->getId()] = maxWeight;
        node->addNeighbor(neighbor, maxWeight);
    }
}

void WSTC_controller::init_grid(int **oldGrid, int **oldWeights, int D, int _width, int _height) {
    int gridWidth = _width / D;
    int gridHeight = _height / D;

    int **grid = new int *[gridWidth];
    int **weightsGrid = new int *[gridWidth];
    for (int i = 0; i < gridWidth; i++) {
        grid[i] = new int[gridHeight];
        weightsGrid[i] = new int[gridHeight];
    }

    int maxWeight;
    for (int i = 0; i < gridHeight; i++) {
        for (int j = 0; j < gridWidth; j++) {
            grid[i][j] = 0;
            maxWeight = 0;
            weightsGrid[i][j] = 0;
            for (int k = 0; k < D; k++) {
                for (int m = 0; m < D; m++) {
                    if (oldWeights[i * D + k][j * D + m] > maxWeight) {
                        maxWeight = oldWeights[i * D + k][j * D + m];
                    }
                }
            }
            weightsGrid[i][j] = maxWeight;
        }
    }

    if (_height % 2 != 0) {
        _height -= 1;
    }

    if (_width % 2 != 0) {
        _width -= 1;
    }

    for (int i = 0; i < _height; i++) {
        for (int j = 0; j < _width; j++) {
            if (oldGrid[i][j] == 1) {
                grid[i / D][j / D] = 1;
            }

        }
    }

    if (D == robotGridSize) {
        uniformGrid = grid;
        weightsUniform = weightsGrid;
    } else if (D == 2) {
        coarseGrid = grid;
        weightsCoarse = weightsGrid;
    }

}

void WSTC_controller::pos_to_col_row(CVector2 pos, int *pCol, int *pRow) {
    *pCol = (pos.GetX() - origin.GetX()) / resolution;
    *pRow = (pos.GetY() - origin.GetY()) / resolution;
}

void WSTC_controller::pos_to_col_row_uniform(int *pCol, int *pRow) {
    colU = *pCol / robotGridSize;
    rowU = *pRow / robotGridSize;
}

void WSTC_controller::pos_to_col_row_coarse(int *pCol, int *pRow) {
    colC = *pCol / 2;
    rowC = *pRow / 2;
}

bool WSTC_controller::got_to_cell(int _col, int _row) {
    Real threshold = 0.0005;
    CVector2 cell_center_pos;
    cell_center_pos.Set(_col * resolution, _row * resolution);
    cell_center_pos += origin;
    if ((pos - cell_center_pos).SquareLength() < threshold) {
        return true;
    } else {
        return false;
    }
}

bool WSTC_controller::got_to_orientation(CDegrees degree) {
    Real deg = (degreeX - degree).UnsignedNormalize().GetValue();
    if ((deg > 0.5) && (deg < 359.5)) {
        return false;
    } else {
        return true;
    }
}

void WSTC_controller::save_grid_to_file(string name, int **grid, int _height, int _width) {
    ofstream m_cOutput;
    m_cOutput.open(name, ios_base::trunc | ios_base::out);
    for (int row = _height - 1; row >= 0; row--) {
        for (int col = 0; col < _width; col++) {
            m_cOutput << grid[row][col] << " ";
        }
        m_cOutput << endl;
    }
    m_cOutput.close();
}

void WSTC_controller::save_grid_to_file_with_robot_location(string name, int **grid,
                                                            int _height, int _width,
                                                            int robot_col, int robot_row) {
    ofstream m_cOutput;
    m_cOutput.open(name, ios_base::trunc | ios_base::out);
    int to_print;
    for (int row = _height - 1; row >= 0; row--) {
        for (int col = 0; col < _width; col++) {
            if ((col == robot_col) && (row == robot_row)) {
                to_print = 2;
            } else {
                to_print = grid[row][col];
            }
            m_cOutput << to_print << " ";
        }
        m_cOutput << endl;
    }
    m_cOutput.close();
}

void WSTC_controller::save_nodes_to_file(string name, int _height, int _width) {
    ofstream m_cOutput;
    m_cOutput.open(name, ios_base::trunc | ios_base::out);
    for (int row = _height - 1; row >= 0; row--) {
        for (int col = 0; col < _width; col++) {
            int id = nodesMatrix[row][col]->getId();
            m_cOutput << id << " ";
        }
        m_cOutput << endl;
    }
    m_cOutput.close();
}

void WSTC_controller::save_edges_to_file(string name, int _height, int _width) {
    ofstream m_cOutput;
    m_cOutput.open(name, ios_base::trunc | ios_base::out);
    for (int row = 0; row < _height; row++) {
        for (int col = 0; col < _width; col++) {
            m_cOutput << neighborsMatrix[row][col] << " ";
        }
        m_cOutput << endl;
    }
    m_cOutput.close();
}

Node::Node(int
           _id, int
           _x, int
           _y, int
           _weight, bool
           _obstacle) {
    id = _id;
    x = _x;
    y = _y;
    weight = _weight;
    obstacle = _obstacle;
}

int Node::getId() const {
    return id;
}


int Node::getX() const {
    return x;
}

int Node::getY() const {
    return y;
}

int Node::getWeight() const {
    return weight;
}

bool Node::isObstacle() const {
    return obstacle;
}

vector<pair<Node *, int>> Node::getNeighbors() {
    return neighbors;
}

void Node::addNeighbor(Node *n, int weight) {
    neighbors.push_back(make_pair(n, weight));
}