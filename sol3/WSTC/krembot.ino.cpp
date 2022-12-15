#include "krembot.ino.h"

#define INF 9999999

int col, row, colU, rowU, colC, rowC;
int **occupancyGrid;
int **uniformGrid;
int **coarseGrid;
int **weights;
int **weightsUniform;
int **weightsCoarse;
Direction **dirMatrix;
Node ***nodesMatrixCoarse;
Node ***nodesMatrixUni;
int **neighborsMatrix;
Real resolution;
CVector2 origin;
int height, width;
CVector2 pos;
CDegrees degreeX;
int robotGridSize;
vector<pair<int, int>> mst;
vector<Node *> path;
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

    // nodesMatrixUniform
    init_nodes_matrix_uniform(w, h);
    save_nodes_to_file("/home/oriya/krembot_sim/krembot_ws/files/nodesUni.txt", nodesMatrixUni, h, w);

    // nodesMatrixCoarse
    init_nodes_matrix_coarse(w / 2, h / 2);
    save_nodes_to_file("/home/oriya/krembot_sim/krembot_ws/files/nodesCoarse.txt", nodesMatrixCoarse, h / 2, w / 2);

    // neighborsMatrix
    int numOfNodes = (w / 2) * (h / 2);
    init_neighbors_matrix(w / 2, h / 2);
    save_edges_to_file("/home/oriya/krembot_sim/krembot_ws/files/neighbors.txt", numOfNodes, numOfNodes);

    // MST
    prim(numOfNodes);

    // directionsMatrix
    init_directions_matrix(w / 2, h / 2);

    // path
    init_path();

    free_memory();
}

void WSTC_controller::init_path() {
    Node *root = nodesMatrixUni[rowU][colU];
    vector<Node *> blackNodes;
    vector<Node *> neighbors;
    neighbors.push_back(root);
    Node *current;
    int i=0;
    while (i<10) {
        i++;
        current = neighbors.front();
        if (current == nullptr) {
            break;
        }
        path.push_back(current);
        blackNodes.push_back(current);
        neighbors = get_relevant_neighbors(current);
//        neighbors = get_unBlackNodes(neighbors, blackNodes);
    }
    path.push_back(root);
}

vector<Node *> WSTC_controller::get_unBlackNodes(vector<Node *> nodes, vector<Node *> blackNodes) {
    vector<Node *> unBlackNodes;
    bool isExit = false;
    for (int i = 0; i < nodes.size(); i++) {
        for (int j = 0; j < blackNodes.size(); j++) {
            if (nodes[i]->isEqual(blackNodes[j])) {
                isExit = true;
            }
        }
        if (!isExit) {
            unBlackNodes.push_back(nodes[i]);
        }
    }
    return unBlackNodes;
}

vector<Node *> WSTC_controller::get_relevant_neighbors(Node *node) {
    vector<Node *> relevant_neighbors;
    Node *megaNode = nodesMatrixCoarse[node->getX() / 2][node->getY() / 2];
    Direction dir;
    vector<Node *> neighbors = megaNode->getNeighbors();
    for (int i = 0; i < neighbors.size(); i++) {
        if (isExistEdge(megaNode, neighbors[i])) {
            update_directions_matrix(megaNode, neighbors[i]);
        }
    }
    int xRelative = node->getX() % 2;
    int yRelative = node->getY() % 2;
    Direction validDir = dirMatrix[megaNode->getX()][megaNode->getY()];
    // down-left
    if (xRelative == 0 && yRelative == 0) {
        if (validDir.up && !validDir.left) {
            relevant_neighbors.push_back(nodesMatrixUni[node->getX() + 1][node->getY()]);
        }
        if (validDir.right && !validDir.down) {
            relevant_neighbors.push_back(nodesMatrixUni[node->getX()][node->getY() + 1]);
        }
        if (validDir.down) {
            relevant_neighbors.push_back(nodesMatrixUni[node->getX() - 1][node->getY()]);
        }
        if (validDir.left) {
            relevant_neighbors.push_back(nodesMatrixUni[node->getX()][node->getY() - 1]);
        }
    }
        // down-right
    else if (xRelative == 0 && yRelative == 1) {
        if (validDir.up && !validDir.right) {
            relevant_neighbors.push_back(nodesMatrixUni[node->getX() + 1][node->getY()]);
        }
        if (validDir.right) {
            relevant_neighbors.push_back(nodesMatrixUni[node->getX()][node->getY() + 1]);
        }
        if (validDir.down) {
            relevant_neighbors.push_back(nodesMatrixUni[node->getX() - 1][node->getY()]);
        }
        if (validDir.left && !validDir.down) {
            relevant_neighbors.push_back(nodesMatrixUni[node->getX()][node->getY() - 1]);
        }
    }
        // up-left
    else if (xRelative == 1 && yRelative == 0) {
        if (validDir.up) {
            relevant_neighbors.push_back(nodesMatrixUni[node->getX() + 1][node->getY()]);
        }
        if (validDir.right && !validDir.up) {
            relevant_neighbors.push_back(nodesMatrixUni[node->getX()][node->getY() + 1]);
        }
        if (validDir.down && !validDir.left) {
            relevant_neighbors.push_back(nodesMatrixUni[node->getX() - 1][node->getY()]);
        }
        if (validDir.left) {
            relevant_neighbors.push_back(nodesMatrixUni[node->getX()][node->getY() - 1]);
        }
    }
        // up-right
    else if (xRelative == 1 && yRelative == 1) {
        if (validDir.up) {
            relevant_neighbors.push_back(nodesMatrixUni[node->getX() + 1][node->getY()]);
        }
        if (validDir.right) {
            relevant_neighbors.push_back(nodesMatrixUni[node->getX()][node->getY() + 1]);
        }
        if (validDir.down && !validDir.right) {
            relevant_neighbors.push_back(nodesMatrixUni[node->getX() - 1][node->getY()]);
        }
        if (validDir.left && !validDir.up) {
            relevant_neighbors.push_back(nodesMatrixUni[node->getX()][node->getY() - 1]);
        }
    }

    return relevant_neighbors;
}


void WSTC_controller::init_directions_matrix(int _width, int _height) {
    dirMatrix = new Direction *[_width];
    for (int i = 0; i < _width; i++) {
        dirMatrix[i] = new Direction[_height];
        for (int j = 0; j < _height; j++) {
            Direction d = {false, false, false, false};
            dirMatrix[i][j] = d;
        }
    }
}

void WSTC_controller::update_directions_matrix(Node *n1, Node *n2) {
    Direction dir = {false, false, false, false};
    if (n1->getX() > n2->getX()) {
        dir.down = true;
    } else if (n1->getX() < n2->getX()) {
        dir.up = true;
    } else if (n1->getY() < n2->getY()) {
        dir.right = true;
    } else if (n1->getY() > n2->getY()) {
        dir.left = true;
    }
    dirMatrix[n1->getX()][n1->getY()] = dir;
}

bool WSTC_controller::isExistEdge(Node *node1, Node *node2) {
    for (int i = 0; i < mst.size(); i++) {
        if (node1->getId() == mst[i].first && node2->getId() == mst[i].second) {
            return true;
        } else if (node2->getId() == mst[i].first && node1->getId() == mst[i].second) {
            return true;
        }
    }
    return false;
}

void WSTC_controller::prim(int numOfNodes) {
    Node *startNode = nodesMatrixCoarse[rowC][colC];
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
        if (neighborsMatrix[x][y] != -1) {
            LOG << x << " - " << y << " :  " << neighborsMatrix[x][y] << endl;
            mst.push_back(make_pair(x, y));
        }
        selected[y] = true;
        numOfEdges++;
    }
    LOG << mst.size() << endl;
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
            delete nodesMatrixCoarse[i][j];
        }
        delete[] nodesMatrixCoarse[i];
    }
    delete[] nodesMatrixCoarse;

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

void WSTC_controller::init_nodes_matrix_coarse(int _width, int _height) {
    nodesMatrixCoarse = new Node **[_width];
    for (int i = 0; i < _width; i++) {
        nodesMatrixCoarse[i] = new Node *[_height];
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
            nodesMatrixCoarse[i][j] = new Node(id, i, j, weight, isObstacle, true);
            id++;
        }
    }
}

void WSTC_controller::init_nodes_matrix_uniform(int _width, int _height) {
    nodesMatrixUni = new Node **[_width];
    for (int i = 0; i < _width; i++) {
        nodesMatrixUni[i] = new Node *[_height];
    }
    int id = 0;
    bool isObstacle = false;
    for (int i = 0; i < _height; ++i) {
        for (int j = 0; j < _width; j++) {
            if (uniformGrid[i][j] == 1) {
                isObstacle = true;
            } else {
                isObstacle = false;
            }
            nodesMatrixUni[i][j] = new Node(id, i, j, 0, isObstacle, false);
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
            node = nodesMatrixCoarse[i][j];
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
    Node *neighbor = nodesMatrixCoarse[newX][newY];
    int maxWeight = 0;
    if (!neighbor->isObstacle()) {
        maxWeight = max(node->getWeight(), neighbor->getWeight());
        neighborsMatrix[node->getId()][neighbor->getId()] = maxWeight;
        node->addNeighbor(neighbor);
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

void WSTC_controller::save_nodes_to_file(string name, Node ***grid, int _height, int _width) {
    ofstream m_cOutput;
    m_cOutput.open(name, ios_base::trunc | ios_base::out);
    for (int row = _height - 1; row >= 0; row--) {
        for (int col = 0; col < _width; col++) {
            int id = grid[row][col]->getId();
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

Node::Node(int _id, int _x, int _y, int _weight, bool _obstacle, bool _coarseGrid) {
    id = _id;
    x = _x;
    y = _y;
    weight = _weight;
    obstacle = _obstacle;
    coarseGrid = _coarseGrid;
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

vector<Node *> Node::getNeighbors() {
    return neighbors;
}

void Node::addNeighbor(Node *n) {
    neighbors.push_back(n);
}

bool Node::isCoarseNode() const {
    return coarseGrid;
}

bool Node::isEqual(Node *otherNode) {
    if (coarseGrid == otherNode->isCoarseNode() && id == otherNode->getId()) {
        return true;
    }
    return false;
}