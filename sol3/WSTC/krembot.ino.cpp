// Shira Taitelbaum 322207341
// Oriya Yehudai 211544150

#include "krembot.ino.h"

#define INF 9999999

// const degrees of possible directions
const CDegrees upDeg = CDegrees(90);
const CDegrees rightDeg = CDegrees(0);
const CDegrees leftDeg = CDegrees(180);
const CDegrees downDeg = CDegrees(270);
// first location of the robot - col, row - in occupancyGrid, colU, rowU - in uniformGrid, colC, rowC - occupancyGrid
int col, row, colU, rowU, colC, rowC;
// grids and weights
int **occupancyGrid;
int **uniformGrid;
int **coarseGrid;
int **weights;
int **weightsUniform;
int **weightsCoarse;
// matrix of valid directions(edges) of each node in coarseGrid
Direction **dirMatrix;
// nodes matrix of coarseGrid
Node ***nodesMatrixCoarse;
// nodes matrix of uniformGrid
Node ***nodesMatrixUni;
// neighbors matrix of coarseGrid
int **neighborsMatrix;
Real resolution;
CVector2 origin;
int height, width;
CVector2 pos;
CDegrees degreeX;
int robotGridSize;
// minimum spanning tree - contains pairs of ID's of nodes
vector<pair<int, int>> mst;
// the final path of the robot - contains Nodes of the uniform grid
vector<Node *> path;
// count for the loop function
int loopIndex;
// bool variable to indicates if we are in the first cell in loop
bool isFirstCell;


enum State {
    centering_robot_x,
    centering_robot_y,
    move,
    turn,
    stop
} state = centering_robot_x;

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
    loopIndex = 0;
    isFirstCell = true;

    // occupancyGrid
//    save_grid_to_file("/home/oriya/krembot_sim/krembot_ws/WSTC/grid.txt", occupancyGrid, height, width);
    pos = posMsg.pos;
    degreeX = posMsg.degreeX;
    pos_to_col_row(pos, &col, &row);
//    save_grid_to_file_with_robot_location("/home/oriya/krembot_sim/krembot_ws/files/grid-with-robot-loc.txt",
//                                          occupancyGrid, height, width, col, row);


    // initialize uniformGrid and weightsUniform
    init_grid(occupancyGrid, weights, robotGridSize, width, height);
    int h = height / robotGridSize;
    int w = width / robotGridSize;
//    save_grid_to_file("/home/oriya/krembot_sim/krembot_ws/files/uniform-grid.txt", uniformGrid, h, w);
//    save_grid_to_file("/home/oriya/krembot_sim/krembot_ws/files/uniform-weights.txt", weightsUniform, h, w);
    pos_to_col_row_uniform(&col, &row);
//    save_grid_to_file_with_robot_location("/home/oriya/krembot_sim/krembot_ws/files/uniform-with-robot-loc.txt",
//                                          uniformGrid, h, w, colU, rowU);


    // initialize coarseGrid and weights Coarse
    init_grid(uniformGrid, weightsUniform, 2, w, h);
//    save_grid_to_file("/home/oriya/krembot_sim/krembot_ws/files/coarse-grid.txt", coarseGrid, h / 2, w / 2);
//    save_grid_to_file("/home/oriya/krembot_sim/krembot_ws/files/coarse-weights.txt", weightsCoarse, h / 2, w / 2);
    pos_to_col_row_coarse(&colU, &rowU);
//    save_grid_to_file_with_robot_location("/home/oriya/krembot_sim/krembot_ws/files/coarse-with-robot-loc.txt",
//                                          coarseGrid, h / 2, w / 2, colC, rowC);

    // initialize nodesMatrixUniform
    init_nodes_matrix_uniform(w, h);
//    save_nodes_to_file("/home/oriya/krembot_sim/krembot_ws/files/nodesUni.txt", nodesMatrixUni, h, w);

    // initialize nodesMatrixCoarse
    init_nodes_matrix_coarse(w / 2, h / 2);
//    save_nodes_to_file("/home/oriya/krembot_sim/krembot_ws/files/nodesCoarse.txt", nodesMatrixCoarse, h / 2, w / 2);

    // initialize neighborsMatrix
    int numOfNodes = (w / 2) * (h / 2);
    init_neighbors_matrix(w / 2, h / 2);
//    save_edges_to_file("/home/oriya/krembot_sim/krembot_ws/files/neighbors.txt", numOfNodes, numOfNodes);

    // initialize MST
    prim(numOfNodes);

    // initialize directionsMatrix
    init_directions_matrix(w / 2, h / 2);

    // initialize path
    init_path();

    // print tree
//    save_tree_to_file("/home/oriya/krembot_sim/krembot_ws/files/mst.txt", coarseGrid, dirMatrix, h / 2, w / 2);

    free_memory();

}

/// function that initializes the path for the robot movement
void WSTC_controller::init_path() {
    // the first node in the path is the cell of the robot position in the uniform grid
    Node *root = nodesMatrixUni[rowU][colU];
    // blackNodes keeps nodes that already exists in path
    vector<Node *> blackNodes;
    vector<Node *> neighbors;
    neighbors.push_back(root);
    path.push_back(root);
    Node *current = root;
    Node *prev = root;
    while (true) {
        if (neighbors.empty()) {
            // if there are no more neighbors - stop the loop
            break;
        }
        prev = current;
        current = neighbors.front();
        // add current to path
        path.push_back(current);
        // add current to blackNodes
        blackNodes.push_back(current);
        // find the neighbors cells of the current node
        neighbors = get_relevant_neighbors(current);
        // delete from neighbors nodes which already in path
        neighbors = get_unBlackNodes(neighbors, blackNodes);
    }
    // add the root to path - to create a circular path
    path.push_back(root);
}

/// function that returns nodes which are not in the blackNodes vector
vector<Node *> WSTC_controller::get_unBlackNodes(vector<Node *> nodes, vector<Node *> blackNodes) {
    vector<Node *> unBlackNodes;
    bool isExist;
    for (int i = 0; i < nodes.size(); i++) {
        isExist = false;
        // for each node check if it exists in the blacNodes
        for (int j = 0; j < blackNodes.size(); j++) {
            if (nodes[i]->getId() == blackNodes[j]->getId()) {
                // if exits - turn on flag
                isExist = true;
                break;
            }
        }
        if (!isExist) {
            // if not exits - add to unBlackNodes
            unBlackNodes.push_back(nodes[i]);
        }
    }
    return unBlackNodes;
}

/// function that returns the neighbors of a node in the uniform grid according to the mst
vector<Node *> WSTC_controller::get_relevant_neighbors(Node *node) {
    vector<Node *> relevant_neighbors;
    // find the megaCell that corresponds to the node in the coarse grid
    Node *megaNode = nodesMatrixCoarse[node->getX() / 2][node->getY() / 2];
    // find the position of the node in its megaCell
    int xRelative = node->getX() % 2;
    int yRelative = node->getY() % 2;
    // get the valid directions (edges) of the megaCell
    Direction validDir = dirMatrix[megaNode->getX()][megaNode->getY()];

    /*** here we add the neighbors of the node in the uniformGrid by dividing to different cases of the node location
      in its megaCell. In each case, we checked all possible cases of edges in the megaCell in order to calculate
      which neighbors is relevant ***/

    // position of the node in its megaCell is down-left (0,0)
    if (xRelative == 0 && yRelative == 0) {
        // 1000 (1)
        if (validDir.up && !validDir.right && !validDir.down && !validDir.left) {
            relevant_neighbors.push_back(nodesMatrixUni[node->getX() + 1][node->getY()]);
            relevant_neighbors.push_back(nodesMatrixUni[node->getX()][node->getY() + 1]);
        }
        // 0100 (2)
        if (!validDir.up && validDir.right && !validDir.down && !validDir.left) {
            relevant_neighbors.push_back(nodesMatrixUni[node->getX()][node->getY() + 1]);
            relevant_neighbors.push_back(nodesMatrixUni[node->getX() + 1][node->getY()]);
        }
        // 1100 (3)
        if (validDir.up && validDir.right && !validDir.down && !validDir.left) {
            relevant_neighbors.push_back(nodesMatrixUni[node->getX()][node->getY() + 1]);
            relevant_neighbors.push_back(nodesMatrixUni[node->getX() + 1][node->getY()]);
        }
        // 0010 (4)
        if (!validDir.up && !validDir.right && validDir.down && !validDir.left) {
            relevant_neighbors.push_back(nodesMatrixUni[node->getX() - 1][node->getY()]);
            relevant_neighbors.push_back(nodesMatrixUni[node->getX() + 1][node->getY()]);
        }
        // 1010 (5)
        if (validDir.up && !validDir.right && validDir.down && !validDir.left) {
            relevant_neighbors.push_back(nodesMatrixUni[node->getX() - 1][node->getY()]);
            relevant_neighbors.push_back(nodesMatrixUni[node->getX() + 1][node->getY()]);
        }
        // 0110 (6)
        if (!validDir.up && validDir.right && validDir.down && !validDir.left) {
            relevant_neighbors.push_back(nodesMatrixUni[node->getX() - 1][node->getY()]);
            relevant_neighbors.push_back(nodesMatrixUni[node->getX() + 1][node->getY()]);
        }
        // 1110 (7)
        if (validDir.up && validDir.right && validDir.down && !validDir.left) {
            relevant_neighbors.push_back(nodesMatrixUni[node->getX() - 1][node->getY()]);
            relevant_neighbors.push_back(nodesMatrixUni[node->getX() + 1][node->getY()]);
        }
        // 0001 (8)
        if (!validDir.up && !validDir.right && !validDir.down && validDir.left) {
            relevant_neighbors.push_back(nodesMatrixUni[node->getX()][node->getY() - 1]);
            relevant_neighbors.push_back(nodesMatrixUni[node->getX()][node->getY() + 1]);
        }
        // 1001 (9)
        if (validDir.up && !validDir.right && !validDir.down && validDir.left) {
            relevant_neighbors.push_back(nodesMatrixUni[node->getX()][node->getY() - 1]);
            relevant_neighbors.push_back(nodesMatrixUni[node->getX()][node->getY() + 1]);
        }
        // 0101 (10)
        if (!validDir.up && validDir.right && !validDir.down && validDir.left) {
            relevant_neighbors.push_back(nodesMatrixUni[node->getX()][node->getY() - 1]);
            relevant_neighbors.push_back(nodesMatrixUni[node->getX()][node->getY() + 1]);
        }
        // 1101 (11)
        if (validDir.up && validDir.right && !validDir.down && validDir.left) {
            relevant_neighbors.push_back(nodesMatrixUni[node->getX()][node->getY() - 1]);
            relevant_neighbors.push_back(nodesMatrixUni[node->getX()][node->getY() + 1]);
        }
        // 0011 (12)
        if (!validDir.up && !validDir.right && validDir.down && validDir.left) {
            relevant_neighbors.push_back(nodesMatrixUni[node->getX()][node->getY() - 1]);
            relevant_neighbors.push_back(nodesMatrixUni[node->getX() - 1][node->getY()]);
        }
        // 1011 (13)
        if (validDir.up && !validDir.right && validDir.down && validDir.left) {
            relevant_neighbors.push_back(nodesMatrixUni[node->getX()][node->getY() - 1]);
            relevant_neighbors.push_back(nodesMatrixUni[node->getX() - 1][node->getY()]);
        }
        // 0111 (14)
        if (!validDir.up && validDir.right && validDir.down && validDir.left) {
            relevant_neighbors.push_back(nodesMatrixUni[node->getX()][node->getY() - 1]);
            relevant_neighbors.push_back(nodesMatrixUni[node->getX() - 1][node->getY()]);
        }
        // 1111 (15)
        if (validDir.up && validDir.right && validDir.down && validDir.left) {
            relevant_neighbors.push_back(nodesMatrixUni[node->getX()][node->getY() - 1]);
            relevant_neighbors.push_back(nodesMatrixUni[node->getX() - 1][node->getY()]);
        }
    }
        // position of the node in its megaCell is down-right (0,1)
    else if (xRelative == 0 && yRelative == 1) {
        // 1000 (1)
        if (validDir.up && !validDir.right && !validDir.down && !validDir.left) {
            relevant_neighbors.push_back(nodesMatrixUni[node->getX() + 1][node->getY()]);
            relevant_neighbors.push_back(nodesMatrixUni[node->getX()][node->getY() - 1]);
        }
        // 0100 (2)
        if (!validDir.up && validDir.right && !validDir.down && !validDir.left) {
            relevant_neighbors.push_back(nodesMatrixUni[node->getX()][node->getY() + 1]);
            relevant_neighbors.push_back(nodesMatrixUni[node->getX()][node->getY() - 1]);
        }
        // 1100 (3)
        if (validDir.up && validDir.right && !validDir.down && !validDir.left) {
            relevant_neighbors.push_back(nodesMatrixUni[node->getX()][node->getY() + 1]);
            relevant_neighbors.push_back(nodesMatrixUni[node->getX()][node->getY() - 1]);
        }
        // 0010 (4)
        if (!validDir.up && !validDir.right && validDir.down && !validDir.left) {
            relevant_neighbors.push_back(nodesMatrixUni[node->getX() - 1][node->getY()]);
            relevant_neighbors.push_back(nodesMatrixUni[node->getX() + 1][node->getY()]);
        }
        // 1010 (5)
        if (validDir.up && !validDir.right && validDir.down && !validDir.left) {
            relevant_neighbors.push_back(nodesMatrixUni[node->getX() - 1][node->getY()]);
            relevant_neighbors.push_back(nodesMatrixUni[node->getX() + 1][node->getY()]);
        }
        // 0110 (6)
        if (!validDir.up && validDir.right && validDir.down && !validDir.left) {
            relevant_neighbors.push_back(nodesMatrixUni[node->getX() - 1][node->getY()]);
            relevant_neighbors.push_back(nodesMatrixUni[node->getX()][node->getY() + 1]);
        }
        // 1110 (7)
        if (validDir.up && validDir.right && validDir.down && !validDir.left) {
            relevant_neighbors.push_back(nodesMatrixUni[node->getX() - 1][node->getY()]);
            relevant_neighbors.push_back(nodesMatrixUni[node->getX()][node->getY() + 1]);
        }
        // 0001 (8)
        if (!validDir.up && !validDir.right && !validDir.down && validDir.left) {
            relevant_neighbors.push_back(nodesMatrixUni[node->getX()][node->getY() - 1]);
            relevant_neighbors.push_back(nodesMatrixUni[node->getX() + 1][node->getY()]);
        }
        // 1001 (9)
        if (validDir.up && !validDir.right && !validDir.down && validDir.left) {
            relevant_neighbors.push_back(nodesMatrixUni[node->getX()][node->getY() - 1]);
            relevant_neighbors.push_back(nodesMatrixUni[node->getX() + 1][node->getY()]);
        }
        // 0101 (10)
        if (!validDir.up && validDir.right && !validDir.down && validDir.left) {
            relevant_neighbors.push_back(nodesMatrixUni[node->getX()][node->getY() - 1]);
            relevant_neighbors.push_back(nodesMatrixUni[node->getX()][node->getY() + 1]);
        }
        // 1101 (11)
        if (validDir.up && validDir.right && !validDir.down && validDir.left) {
            relevant_neighbors.push_back(nodesMatrixUni[node->getX()][node->getY() - 1]);
            relevant_neighbors.push_back(nodesMatrixUni[node->getX()][node->getY() + 1]);
        }
        // 0011 (12)
        if (!validDir.up && !validDir.right && validDir.down && validDir.left) {
            relevant_neighbors.push_back(nodesMatrixUni[node->getX() - 1][node->getY()]);
            relevant_neighbors.push_back(nodesMatrixUni[node->getX() + 1][node->getY()]);
        }
        // 1011 (13)
        if (validDir.up && !validDir.right && validDir.down && validDir.left) {
            relevant_neighbors.push_back(nodesMatrixUni[node->getX() + 1][node->getY()]);
            relevant_neighbors.push_back(nodesMatrixUni[node->getX() - 1][node->getY()]);
        }
        // 0111 (14)
        if (!validDir.up && validDir.right && validDir.down && validDir.left) {
            relevant_neighbors.push_back(nodesMatrixUni[node->getX()][node->getY() + 1]);
            relevant_neighbors.push_back(nodesMatrixUni[node->getX() - 1][node->getY()]);
        }
        // 1111 (15)
        if (validDir.up && validDir.right && validDir.down && validDir.left) {
            relevant_neighbors.push_back(nodesMatrixUni[node->getX()][node->getY() + 1]);
            relevant_neighbors.push_back(nodesMatrixUni[node->getX() - 1][node->getY()]);
        }
    }
        // position of the node in its megaCell is up-left (1,0)
    else if (xRelative == 1 && yRelative == 0) {
        // 1000 (1)
        if (validDir.up && !validDir.right && !validDir.down && !validDir.left) {
            relevant_neighbors.push_back(nodesMatrixUni[node->getX() + 1][node->getY()]);
            relevant_neighbors.push_back(nodesMatrixUni[node->getX() - 1][node->getY()]);
        }
        // 0100 (2)
        if (!validDir.up && validDir.right && !validDir.down && !validDir.left) {
            relevant_neighbors.push_back(nodesMatrixUni[node->getX()][node->getY() + 1]);
            relevant_neighbors.push_back(nodesMatrixUni[node->getX() - 1][node->getY()]);
        }
        // 1100 (3)
        if (validDir.up && validDir.right && !validDir.down && !validDir.left) {
            relevant_neighbors.push_back(nodesMatrixUni[node->getX() + 1][node->getY()]);
            relevant_neighbors.push_back(nodesMatrixUni[node->getX() - 1][node->getY()]);
        }
        // 0010 (4)
        if (!validDir.up && !validDir.right && validDir.down && !validDir.left) {
            relevant_neighbors.push_back(nodesMatrixUni[node->getX() - 1][node->getY()]);
            relevant_neighbors.push_back(nodesMatrixUni[node->getX()][node->getY() + 1]);
        }
        // 1010 (5)
        if (validDir.up && !validDir.right && validDir.down && !validDir.left) {
            relevant_neighbors.push_back(nodesMatrixUni[node->getX() - 1][node->getY()]);
            relevant_neighbors.push_back(nodesMatrixUni[node->getX() + 1][node->getY()]);
        }
        // 0110 (6)
        if (!validDir.up && validDir.right && validDir.down && !validDir.left) {
            relevant_neighbors.push_back(nodesMatrixUni[node->getX() - 1][node->getY()]);
            relevant_neighbors.push_back(nodesMatrixUni[node->getX()][node->getY() + 1]);
        }
        // 1110 (7)
        if (validDir.up && validDir.right && validDir.down && !validDir.left) {
            relevant_neighbors.push_back(nodesMatrixUni[node->getX() - 1][node->getY()]);
            relevant_neighbors.push_back(nodesMatrixUni[node->getX() + 1][node->getY()]);
        }
        // 0001 (8)
        if (!validDir.up && !validDir.right && !validDir.down && validDir.left) {
            relevant_neighbors.push_back(nodesMatrixUni[node->getX()][node->getY() + 1]);
            relevant_neighbors.push_back(nodesMatrixUni[node->getX()][node->getY() - 1]);
        }
        // 1001 (9)
        if (validDir.up && !validDir.right && !validDir.down && validDir.left) {
            relevant_neighbors.push_back(nodesMatrixUni[node->getX()][node->getY() - 1]);
            relevant_neighbors.push_back(nodesMatrixUni[node->getX() + 1][node->getY()]);
        }
        // 0101 (10)
        if (!validDir.up && validDir.right && !validDir.down && validDir.left) {
            relevant_neighbors.push_back(nodesMatrixUni[node->getX()][node->getY() - 1]);
            relevant_neighbors.push_back(nodesMatrixUni[node->getX()][node->getY() + 1]);
        }
        // 1101 (11)
        if (validDir.up && validDir.right && !validDir.down && validDir.left) {
            relevant_neighbors.push_back(nodesMatrixUni[node->getX()][node->getY() - 1]);
            relevant_neighbors.push_back(nodesMatrixUni[node->getX() + 1][node->getY()]);
        }
        // 0011 (12)
        if (!validDir.up && !validDir.right && validDir.down && validDir.left) {
            relevant_neighbors.push_back(nodesMatrixUni[node->getX()][node->getY() - 1]);
            relevant_neighbors.push_back(nodesMatrixUni[node->getX()][node->getY() + 1]);
        }
        // 1011 (13)
        if (validDir.up && !validDir.right && validDir.down && validDir.left) {
            relevant_neighbors.push_back(nodesMatrixUni[node->getX() + 1][node->getY()]);
            relevant_neighbors.push_back(nodesMatrixUni[node->getX()][node->getY() - 1]);
        }
        // 0111 (14)
        if (!validDir.up && validDir.right && validDir.down && validDir.left) {
            relevant_neighbors.push_back(nodesMatrixUni[node->getX()][node->getY() + 1]);
            relevant_neighbors.push_back(nodesMatrixUni[node->getX()][node->getY() - 1]);
        }
        // 1111 (15)
        if (validDir.up && validDir.right && validDir.down && validDir.left) {
            relevant_neighbors.push_back(nodesMatrixUni[node->getX()][node->getY() - 1]);
            relevant_neighbors.push_back(nodesMatrixUni[node->getX() + 1][node->getY()]);
        }
    }
        // position of the node in its megaCell is up-right (1,1)
    else if (xRelative == 1 && yRelative == 1) {
        // 1000 (1)
        if (validDir.up && !validDir.right && !validDir.down && !validDir.left) {
            relevant_neighbors.push_back(nodesMatrixUni[node->getX() + 1][node->getY()]);
            relevant_neighbors.push_back(nodesMatrixUni[node->getX() - 1][node->getY()]);
        }
        // 0100 (2)
        if (!validDir.up && validDir.right && !validDir.down && !validDir.left) {
            relevant_neighbors.push_back(nodesMatrixUni[node->getX()][node->getY() + 1]);
            relevant_neighbors.push_back(nodesMatrixUni[node->getX()][node->getY() - 1]);
        }
        // 1100 (3)
        if (validDir.up && validDir.right && !validDir.down && !validDir.left) {
            relevant_neighbors.push_back(nodesMatrixUni[node->getX() + 1][node->getY()]);
            relevant_neighbors.push_back(nodesMatrixUni[node->getX()][node->getY() + 1]);
        }
        // 0010 (4)
        if (!validDir.up && !validDir.right && validDir.down && !validDir.left) {
            relevant_neighbors.push_back(nodesMatrixUni[node->getX() - 1][node->getY()]);
            relevant_neighbors.push_back(nodesMatrixUni[node->getX()][node->getY() - 1]);
        }
        // 1010 (5)
        if (validDir.up && !validDir.right && validDir.down && !validDir.left) {
            relevant_neighbors.push_back(nodesMatrixUni[node->getX() - 1][node->getY()]);
            relevant_neighbors.push_back(nodesMatrixUni[node->getX() + 1][node->getY()]);
        }
        // 0110 (6)
        if (!validDir.up && validDir.right && validDir.down && !validDir.left) {
            relevant_neighbors.push_back(nodesMatrixUni[node->getX()][node->getY() + 1]);
            relevant_neighbors.push_back(nodesMatrixUni[node->getX()][node->getY() - 1]);
        }
        // 1110 (7)
        if (validDir.up && validDir.right && validDir.down && !validDir.left) {
            relevant_neighbors.push_back(nodesMatrixUni[node->getX()][node->getY() + 1]);
            relevant_neighbors.push_back(nodesMatrixUni[node->getX() + 1][node->getY()]);
        }
        // 0001 (8)
        if (!validDir.up && !validDir.right && !validDir.down && validDir.left) {
            relevant_neighbors.push_back(nodesMatrixUni[node->getX()][node->getY() - 1]);
            relevant_neighbors.push_back(nodesMatrixUni[node->getX() - 1][node->getY()]);
        }
        // 1001 (9)
        if (validDir.up && !validDir.right && !validDir.down && validDir.left) {
            relevant_neighbors.push_back(nodesMatrixUni[node->getX() + 1][node->getY()]);
            relevant_neighbors.push_back(nodesMatrixUni[node->getX() - 1][node->getY()]);
        }
        // 0101 (10)
        if (!validDir.up && validDir.right && !validDir.down && validDir.left) {
            relevant_neighbors.push_back(nodesMatrixUni[node->getX()][node->getY() - 1]);
            relevant_neighbors.push_back(nodesMatrixUni[node->getX()][node->getY() + 1]);
        }
        // 1101 (11)
        if (validDir.up && validDir.right && !validDir.down && validDir.left) {
            relevant_neighbors.push_back(nodesMatrixUni[node->getX()][node->getY() + 1]);
            relevant_neighbors.push_back(nodesMatrixUni[node->getX() + 1][node->getY()]);
        }
        // 0011 (12)
        if (!validDir.up && !validDir.right && validDir.down && validDir.left) {
            relevant_neighbors.push_back(nodesMatrixUni[node->getX()][node->getY() - 1]);
            relevant_neighbors.push_back(nodesMatrixUni[node->getX() - 1][node->getY()]);
        }
        // 1011 (13)
        if (validDir.up && !validDir.right && validDir.down && validDir.left) {
            relevant_neighbors.push_back(nodesMatrixUni[node->getX() + 1][node->getY()]);
            relevant_neighbors.push_back(nodesMatrixUni[node->getX() - 1][node->getY()]);
        }
        // 0111 (14)
        if (!validDir.up && validDir.right && validDir.down && validDir.left) {
            relevant_neighbors.push_back(nodesMatrixUni[node->getX()][node->getY() + 1]);
            relevant_neighbors.push_back(nodesMatrixUni[node->getX()][node->getY() - 1]);
        }
        // 1111 (15)
        if (validDir.up && validDir.right && validDir.down && validDir.left) {
            relevant_neighbors.push_back(nodesMatrixUni[node->getX()][node->getY() + 1]);
            relevant_neighbors.push_back(nodesMatrixUni[node->getX() + 1][node->getY()]);
        }
    }
    return relevant_neighbors;
}

/// function that initializes the dirMatrix
void WSTC_controller::init_directions_matrix(int _width, int _height) {
    vector<Node *> neighbors;
    dirMatrix = new Direction *[_width];
    for (int i = 0; i < _width; i++) {
        dirMatrix[i] = new Direction[_height];
        // fill all the matrix cells with Direction object that initialized to false
        for (int j = 0; j < _height; j++) {
            Direction d = {false, false, false, false};
            dirMatrix[i][j] = d;
        }
    }
    for (int i = 0; i < _width; i++) {
        for (int j = 0; j < _height; j++) {
            neighbors = nodesMatrixCoarse[i][j]->getNeighbors();
            // run over the neighbors of a specific node
            for (int k = 0; k < neighbors.size(); k++) {
                if (isExistEdge(nodesMatrixCoarse[i][j], neighbors[k])) {
                    // if there is an edge in the mst between the specific node and its neighbor - update the dirMatrix
                    update_directions_matrix(nodesMatrixCoarse[i][j], neighbors[k]);
                }
            }
        }
    }
}

/// function that updates the dirMatrix according to two nodes
void WSTC_controller::update_directions_matrix(Node *n1, Node *n2) {
    // n2 is below n1
    if (n1->getX() > n2->getX()) {
        dirMatrix[n1->getX()][n1->getY()].down = true;
    }
    // n2 is above n1
    if (n1->getX() < n2->getX()) {
        dirMatrix[n1->getX()][n1->getY()].up = true;
    }
    // n2 is to the right of n1
    if (n1->getY() < n2->getY()) {
        dirMatrix[n1->getX()][n1->getY()].right = true;
    }
    // n2 is to the left of n1
    if (n1->getY() > n2->getY()) {
        dirMatrix[n1->getX()][n1->getY()].left = true;
    }
}

/// function that returns true if there is an edge between two nodes in the mst, and false otherwise
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

/// function that initializes the mst according to prim algorithm
void WSTC_controller::prim(int numOfNodes) {
    // startNode is the cell of the robot location on the coarse grid
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
            // add an edge to the mst as a pair of nodes' ID's
            mst.push_back(make_pair(x, y));
        }
        selected[y] = true;
        numOfEdges++;
    }
    delete[] selected;
}

/// function that frees all the memory which was allocated dynamically
void WSTC_controller::free_memory() {
    // free uniformGrid and weightsUniform
    for (int i = 0; i < width / robotGridSize; i++) {
        delete[] uniformGrid[i];
        delete[] weightsUniform[i];
    }
    delete[] uniformGrid;
    delete[] weightsUniform;

    // free coarseGrid and weightsCoarse and dirMatrix
    for (int i = 0; i < width / robotGridSize / 2; i++) {
        delete[] coarseGrid[i];
        delete[] weightsCoarse[i];
        delete[] dirMatrix[i];
    }
    delete[] coarseGrid;
    delete[] weightsCoarse;
    delete[] dirMatrix;

    // free nodesMatrixCoarse
    for (int i = 0; i < width / robotGridSize / 2; i++) {
        for (int j = 0; j < height / robotGridSize / 2; j++) {
            delete nodesMatrixCoarse[i][j];
        }
        delete[] nodesMatrixCoarse[i];
    }
    delete[] nodesMatrixCoarse;

    // free neighborsMatrix
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
    Node *current = nullptr;
    Node *next = nullptr;
    CDegrees deg;
    int angularSpd,xRelative, yRelative;
    if (loopIndex < (path.size() - 1)) {
        // this handles the case that the robot didn't cover all the path nodes
        current = path[loopIndex];
        next = path[loopIndex + 1];
        // find the x and y of the original grid
        yRelative = next->getY() * robotGridSize + robotGridSize / 2;
        xRelative = next->getX() * robotGridSize + robotGridSize / 2;
    } else {
        // this handles the case that the robot covered all the path nodes and needs to stop
        state = State::stop;
    }
    switch (state) {
        case State::centering_robot_x: {
            if (row == xRelative) {
                // if the robot is already in the center row - no need to move it
                state = State::centering_robot_y;
                break;
            } else {
                deg = calc_deg_centering_x(row, xRelative);
            }
            if (!got_to_orientation(deg)) {
                krembot.Base.drive(0, 20);
            } else {
                krembot.Base.stop();
                if (!got_to_cell(col, xRelative)) {
                    krembot.Base.drive(100, 0);
                } else {
                    krembot.Base.stop();
                    pos_to_col_row(pos, &col, &row);
                    state = State::centering_robot_y;
                }
            }
            break;
        }
        case State::centering_robot_y: {
            if (col == yRelative) {
                // if the robot is already in the center col - no need to move it
                state = State::turn;
                break;
            } else {
                deg = calc_deg_centering_y(col, yRelative);
            }
            if (!got_to_orientation(deg)) {
                krembot.Base.drive(0, 20);
            } else {
                krembot.Base.stop();
                if (!got_to_cell(yRelative, row)) {
                    krembot.Base.drive(100, 0);
                } else {
                    krembot.Base.stop();
                    pos_to_col_row(pos, &col, &row);
                    state = State::turn;
                }
            }
            break;
        }
        case State::move: {
            if (!got_to_cell(yRelative, xRelative)) {
                // if the robot didn't get to cell - continue to move
                krembot.Base.drive(100, 0);
            } else {
                // if the robot got to cell - stop and switch to turn state
                krembot.Base.stop();
                loopIndex++;
                krembot.Led.write(255, 0, 0);
                state = State::turn;
            }
            break;
        }
        case State::turn: {
            // calculate the degree that the robot needs to move in
            deg = calcDeg(current, next);
            if (!got_to_orientation(deg)) {
                // if the robot didn't get to the degree - calculate the angularSpd and turn
                angularSpd = calc_Angular_spd(deg);
                krembot.Base.drive(0, angularSpd);
            } else {
                // if the robot got to the degree - stop and switch to move state
                krembot.Base.stop();
                krembot.Led.write(0, 255, 0);
                state = State::move;
            }
            break;
        }
        case State::stop: {
            // stop
            krembot.Led.write(255, 0, 0);
            krembot.Base.stop();
        }
    }
}

/// function that calculates the Angular spd that the robot needs to turn
int WSTC_controller::calc_Angular_spd(CDegrees deg) {
    int angularSpd;

    // general case
    if (deg < degreeX.UnsignedNormalize()) {
        angularSpd = -25;
    } else {
        angularSpd = 25;
    }

    // specific cases - to make turn more efficient
    if (degreeX.UnsignedNormalize() > CDegrees(359.50) && deg == upDeg) {
        angularSpd = 25;
    } else if (degreeX.UnsignedNormalize() < CDegrees(0.5) && deg == downDeg) {
        angularSpd = -25;
    } else if (degreeX.UnsignedNormalize() > CDegrees(269.5) && deg == rightDeg) {
        angularSpd = 25;
    }
    return angularSpd;
}

/// function that calculates the degree that the robot needs to move in order to be in the center
CDegrees WSTC_controller::calc_deg_centering_x(int curr_row, int center_row) {
    // up
    if (curr_row < center_row) {
        return upDeg;
    }
        // down
    else {
        return downDeg;
    }
}

/// function that calculates the degree that the robot needs to move in order to be in the center
CDegrees WSTC_controller::calc_deg_centering_y(int curr_col, int center_col) {
    // right
    if (curr_col < center_col) {
        return rightDeg;
    }
        // left
    else {
        return leftDeg;
    }
}

/// function that calculates the degree that the robot needs to move in
CDegrees WSTC_controller::calcDeg(Node *current, Node *next) {
    // up
    if (current->getX() < next->getX()) {
        return upDeg;
    }
        // right
    else if (current->getY() < next->getY()) {
        return rightDeg;
    }
        // down
    else if (current->getX() > next->getX()) {
        return downDeg;
    }
        // left
    else if (current->getY() > next->getY()) {
        return leftDeg;
    }
        // never reaches this condition
    else {
        return rightDeg;
    }
}

/// function that initializes nodes matrix for the coarse grid
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
                // this is the case of an obstacle
                isObstacle = true;
            } else {
                isObstacle = false;
            }
            weight = weightsCoarse[i][j];
            // create a new object of node
            nodesMatrixCoarse[i][j] = new Node(id, i, j, weight, isObstacle, true);
            id++;
        }
    }
}

/// function that initializes nodes matrix for the uniform grid
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
                // this is the case of an obstacle
                isObstacle = true;
            } else {
                isObstacle = false;
            }
            // create a new object of node
            nodesMatrixUni[i][j] = new Node(id, i, j, 0, isObstacle, false);
            id++;
        }
    }
}

/// function that initializes neighbors matrix for the coarse grid
void WSTC_controller::init_neighbors_matrix(int _width, int _height) {
    int numOfNodes = _width * _height;
    neighborsMatrix = new int *[numOfNodes];
    for (int i = 0; i < numOfNodes; i++) {
        neighborsMatrix[i] = new int[numOfNodes];
    }

    // fill all the matrix cells with (-1)
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
                // if node is not an obstacle - add his edges and neighbors
                add_edge(node, _width, _height);
            }
        }
    }
}

/// function that adds neighbors to a specific node
void WSTC_controller::add_edge(Node *node, int _width, int _height) {
    int id = node->getId();
    int x = node->getX();
    int y = node->getY();
    int weight = node->getWeight();
    int newX, newY, maxWeight;
    Node *neighbor;

    // neighbor from up
    newX = x + 1;
    newY = y;
    // check we don't get out of bounds of grid
    if (newX < _height) {
        check_valid_edge(newX, newY, node);
    }


    // neighbor from down
    newX = x - 1;
    newY = y;
    // check we don't get out of bounds of grid
    if (newX >= 0) {
        check_valid_edge(newX, newY, node);
    }

    // neighbor from right
    newX = x;
    newY = y + 1;
    // check we don't get out of bounds of grid
    if (newY < _width) {
        check_valid_edge(newX, newY, node);
    }

    // neighbor from left
    newX = x;
    newY = y - 1;
    // check we don't get out of bounds of grid
    if (newY >= 0) {
        check_valid_edge(newX, newY, node);
    }
}

/// function that adds an edge if it is valid
void WSTC_controller::check_valid_edge(int newX, int newY, Node *node) {
    Node *neighbor = nodesMatrixCoarse[newX][newY];
    int maxWeight = 0;
    // if neighbor is not an obstacle
    if (!neighbor->isObstacle()) {
        // the weight of an edge is the maximum of weight of the edge's nodes
        maxWeight = max(node->getWeight(), neighbor->getWeight());
        // fill the neighbors matrix with the edge weight to represent that the edge's nodes are neighbors
        neighborsMatrix[node->getId()][neighbor->getId()] = maxWeight;
        // add the neighbor to the neighbors list
        node->addNeighbor(neighbor);
    }
}

/// function that initializes grid and its weights grid
void WSTC_controller::init_grid(int **oldGrid, int **oldWeights, int D, int _width, int _height) {
    // width and height of the new grid
    int gridWidth = _width / D;
    int gridHeight = _height / D;

    int **grid = new int *[gridWidth];
    int **weightsGrid = new int *[gridWidth];
    for (int i = 0; i < gridWidth; i++) {
        grid[i] = new int[gridHeight];
        weightsGrid[i] = new int[gridHeight];
    }

    // fill the weightsGrid - the weight of a cell in the grid is the maximum weight of the cells from the old grid
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

    // if _height or _width is odd
    if (_height % 2 != 0) {
        _height -= 1;
    }
    if (_width % 2 != 0) {
        _width -= 1;
    }

    // create the obstacles in the new grid
    for (int i = 0; i < _height; i++) {
        for (int j = 0; j < _width; j++) {
            if (oldGrid[i][j] == 1) {
                grid[i / D][j / D] = 1;
            }

        }
    }

    if (D == robotGridSize) {
        // then it's the case of uniformGrid
        uniformGrid = grid;
        weightsUniform = weightsGrid;
    } else if (D == 2) {
        // then it's the case of coarseGrid
        coarseGrid = grid;
        weightsCoarse = weightsGrid;
    }

}

/// function that converts the position of the robot from the occupancy grid into a col and a row (showed in the tirgul)
void WSTC_controller::pos_to_col_row(CVector2 Cpos, int *pCol, int *pRow) {
    *pCol = (Cpos.GetX() - origin.GetX()) / resolution;
    *pRow = (Cpos.GetY() - origin.GetY()) / resolution;
}

/// function that converts the location of the robot from the occupancy grid to the uniform grid
void WSTC_controller::pos_to_col_row_uniform(int *pCol, int *pRow) {
    colU = *pCol / robotGridSize;
    rowU = *pRow / robotGridSize;
}

/// function that converts the location of the robot from the uniform grid to the coarse grid (showed in the tirgul)
void WSTC_controller::pos_to_col_row_coarse(int *pCol, int *pRow) {
    colC = *pCol / 2;
    rowC = *pRow / 2;
}

/// function that returns true if the robot got to a wanted cell, and false o.w (showed in the tirgul)
bool WSTC_controller::got_to_cell(int _col, int _row) {
    Real threshold = 0.0009;
    CVector2 cell_center_pos;
    cell_center_pos.Set(_col * resolution, _row * resolution);
    cell_center_pos += origin;
    if ((pos - cell_center_pos).SquareLength() < threshold) {
        return true;
    } else {
        return false;
    }
}

/// function that returns true if the robot got to a wanted degree, and false o.w (showed in the tirgul)
bool WSTC_controller::got_to_orientation(CDegrees degree) {
    Real deg = (degreeX - degree).UnsignedNormalize().GetValue();
    if ((deg > 0.5) && (deg < 359.5)) {
        return false;
    } else {
        return true;
    }
}

/// function that saves a txt file of a grid (for debug, showed in the tirgul)
void WSTC_controller::save_grid_to_file(string name, int **grid, int _height, int _width) {
    ofstream m_cOutput;
    m_cOutput.open(name, ios_base::trunc | ios_base::out);
    for (int i = _height - 1; i >= 0; i--) {
        for (int j = 0; j < _width; j++) {
            m_cOutput << grid[i][j] << " ";
        }
        m_cOutput << endl;
    }
    m_cOutput.close();
}

/// function that saves a txt file of a grid with robot location (for debug, showed in the tirgul)
void WSTC_controller::save_grid_to_file_with_robot_location(string name, int **grid,
                                                            int _height, int _width,
                                                            int robot_col, int robot_row) {
    ofstream m_cOutput;
    m_cOutput.open(name, ios_base::trunc | ios_base::out);
    int to_print;
    for (int i = _height - 1; i >= 0; i--) {
        for (int j = 0; j < _width; j++) {
            if ((j == robot_col) && (i == robot_row)) {
                to_print = 2;
            } else {
                to_print = grid[i][j];
            }
            m_cOutput << to_print << " ";
        }
        m_cOutput << endl;
    }
    m_cOutput.close();
}

/// function that saves a txt file of a nodes gris (for debug)
void WSTC_controller::save_nodes_to_file(string name, Node ***grid, int _height, int _width) {
    ofstream m_cOutput;
    m_cOutput.open(name, ios_base::trunc | ios_base::out);
    for (int i = _height - 1; i >= 0; i--) {
        for (int j = 0; j < _width; j++) {
            int id = grid[i][j]->getId();
            m_cOutput << id << " ";
        }
        m_cOutput << endl;
    }
    m_cOutput.close();
}

/// function that saves a txt file of the mst (for debug)
void WSTC_controller::save_tree_to_file(string name, int **grid, Direction **dir, int _height, int _width) {
    ofstream m_cOutput;
    m_cOutput.open(name, ios_base::trunc | ios_base::out);
    for (int i = _height - 1; i >= 0; i--) {
        for (int j = 0; j < _width; j++) {
            if (dir[i][j].up) {
                m_cOutput << " " << "|" << " ";
            } else {
                m_cOutput << " " << " " << " ";
            }
        }
        m_cOutput << "\n";
        for (int j = 0; j < _width; j++) {
            if (dir[i][j].left) {
                m_cOutput << "-";
            } else {
                m_cOutput << " ";
            }
            m_cOutput << grid[i][j];
            if (dir[i][j].right) {
                m_cOutput << "-";
            } else {
                m_cOutput << " ";
            }
        }
        m_cOutput << "\n";
        for (int j = 0; j < _width; j++) {
            if (dir[i][j].down) {
                m_cOutput << " " << "|" << " ";
            } else {
                m_cOutput << " " << " " << " ";
            }
        }
        m_cOutput << endl;
    }
    m_cOutput.close();
}

/// function that saves a txt file of the neighborsMatrix (for debug)
void WSTC_controller::save_edges_to_file(string name, int _height, int _width) {
    ofstream m_cOutput;
    m_cOutput.open(name, ios_base::trunc | ios_base::out);
    for (int i = 0; i < _height; i++) {
        for (int j = 0; j < _width; j++) {
            m_cOutput << neighborsMatrix[i][j] << " ";
        }
        m_cOutput << endl;
    }
    m_cOutput.close();
}

// constructor
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