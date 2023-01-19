#include "krembot.ino.h"
#include <vector>
#include <algorithm>
#include <cmath>

#define rgb_num 255
#define dis_threshold 10
#define start_time 400
#define block_time 8000
#define add_pos_time 15000


CVector2 pos;
CDegrees degreeX;
vector<CVector2> homePos;
vector<CVector2> homeArea;
float _distanceF, _distanceFL, _distanceFR;
int homeBaseColor, opponentBaseColor, homeTeamColor, opponentTeamColor;
int colorF, colorFL, colorFR, colorR, colorL;
bool increaseTime;
bool homeBotF;
bool oppBotF;
bool isObstcle, isRobot;
bool baseF, baseR, baseL;
bool oppBaseF, oppBaseR, oppBaseL;
bool isFirstBlock;
bool isCollision;
BumpersRes bumpers;
CVector2 blockPos;
int _time;

void foraging_8_controller::setup() {
    krembot.setup();
    writeTeamColor();
    teamName = "foraging_8_controller";
    init_colors();
    increaseTime = false;
    isFirstBlock = true;
    direction = 1;
    _time = start_time;
    frequencyTurnTimer.start(_time);
    startCover.start(1000000);
}

void foraging_8_controller::loop() {
    krembot.loop();
    pos = posMsg.pos;
    degreeX = posMsg.degreeX;
    int angularSpd;

    // update colors and states
    read_colors();
    init_environment_states();

    // restart frequencyTurnTimer
    if (startCover.finished()) {
        _time = start_time;
        frequencyTurnTimer.start(_time);
        startCover.start(1000000);
    }

    switch (state) {

        case State::move: {
//            LOG << "move" << endl;
            // if the robot has food - change state to findBase
            if (hasFood) {
                state = State::findBase;
            }
                // if the robot sees the opponent base in front
            else if (oppBaseF && frequencyBlockTimer.finished()) {
//                LOG << "oppBaseF(move)" << endl;
                krembot.Base.drive(100, 0);
                blockTimer.start(block_time);
                state = State::block;
            }
                // if the robot sees the opponent base from right
            else if (oppBaseR && frequencyBlockTimer.finished()) {
//                LOG << "oppBaseR(move)" << endl;
                krembot.Base.drive(30, -1 * turning_speed);
                blockTimer.start(block_time);
                state = State::block;
            }
                // if the robot sees the opponent base from left
            else if (oppBaseL && frequencyBlockTimer.finished()) {
//                LOG << "oppBaseL(move)" << endl;
                krembot.Base.drive(30, turning_speed);
                blockTimer.start(block_time);
                state = State::block;
            }
                // if the robot sees its base in front
            else if (baseF && frequencyAddPosTimer.finished()) {
//                LOG << "baseF(move)" << endl;
                krembot.Base.drive(100, 0);
                state = State::addHomeBase;
            }
                // if the robot sees its base from right
            else if (baseR && frequencyAddPosTimer.finished()) {
//                LOG << "baseR(move)" << endl;
                krembot.Base.drive(30, -1 * turning_speed);
                state = State::addHomeBase;
            }
                // if the robot sees its base from left
            else if (baseL && frequencyAddPosTimer.finished()) {
//                LOG << "baseL(move)" << endl;
                krembot.Base.drive(30, turning_speed);
                state = State::addHomeBase;
            }
                // if the robot is in a collision
            else if (isCollision) {
//                LOG << "bumped" << endl;
                turnTimer.start(440);
                state = State::collision;
            }
                // if the robot sees a robot in front
            else if (isRobot) {
//                LOG << "isRobot (move)" << endl;
                turnTimer.start(200);
                state = State::passRobot;
            }
                // if the robot sees an obstacle in front
            else if (isObstcle) {
//                LOG << "obstcle (move)" << endl;
                turnTimer.start(200);
                state = State::turn;
            }
                // if the frequencyTurnTimer is finished go to turn state
            else if (frequencyTurnTimer.finished()) {
//                LOG << "frequencyTurnTimer.finished() (move)" << endl;
                turnTimer.start(220);
                state = State::turn;
            }
                // else - drive
            else {
//                LOG << "else (move)" << endl;
                krembot.Base.drive(100, 0);
            }
            break;
        }

        case State::findBase: {
//            LOG << "findBase" << endl;
            // if the robot doesn't have food anymore - change to move state
            if (!hasFood) {
//                LOG << "not hasFood(findBase)" << endl;
                krembot.Base.stop();
                addHomePos();
                frequencyAddPosTimer.start(add_pos_time);
                state = State::move;
                startCover.start(5000);
            }
                // if the robot sees our base in front
            else if (baseF) {
//                LOG << "baseF(findBase)" << endl;
                krembot.Base.drive(100, 0);
            }
                // if the robot sees our base from right
            else if (baseR) {
//                LOG << "baseR(findBase)" << endl;
                krembot.Base.drive(30, -1 * turning_speed);
            }
                // if the robot sees our base from left
            else if (baseL) {
//                LOG << "baseL(findBase)" << endl;
                krembot.Base.drive(30, turning_speed);
            }
                // if the robot sees an obstacle in front
            else if (isObstcle) {
//                LOG << "isObstcle(findBase)" << endl;
                if (_distanceFL < _distanceFR) {
                    direction = -1;
                } else {
                    direction = 1;
                }
                krembot.Base.drive(50, direction * turning_speed);
            } else if (isRobot) {
                krembot.Base.drive(100, 0);
            }
                // else - search our base
            else {
//                LOG << "else(findBase)" << endl;
                // if the robot doesn't know the position of our base
                if (homePos.empty() && homeArea.empty()) {
//                    LOG << "homePos & homeArea empty(findBase)" << endl;
                    krembot.Base.drive(100, 0);
                }
                    // else - the robot knows the position of our base or an area which is close to the base
                else {
//                    LOG << "else else(findBase)" << endl;
                    turnTimer.start(200);
                    state = State::turn;
                }
            }
            break;
        }

        case State::turn: {
//            LOG << "turn" << endl;
            // if the robot has food and knows where our base is
            if (hasFood && (!homePos.empty() || !homeArea.empty())) {
//                LOG << "hasFood(turn)" << endl;
                CVector2 closestBase = find_closest_base();
                CDegrees deg = calculateDeg(closestBase).UnsignedNormalize();
                if (got_to_orientation(deg)) {
                    krembot.Base.drive(100, 0);
                    state = State::findBase;
                } else {
                    angularSpd = calc_Angular_spd(deg);
                    krembot.Base.drive(0, angularSpd);
                }
            }
                // if the robot sees on obstacle in front
            else if (isObstcle) {
//                LOG << "isObstcle(turn)" << endl;
                krembot.Base.drive(0, turning_speed);
                state = State::move;
            }
                // if turnTimer is finished change state to move
            else if (turnTimer.finished()) {
//                LOG << "turnTimer.finished (turn)" << endl;
                // every some steps of the robot increase the time that he will stay in move state
                if (increaseTime) {
                    _time = _time + 400;
                    increaseTime = false;
                } else {
                    increaseTime = true;
                }
                frequencyTurnTimer.start(_time);
                state = State::move;
            }
                // else - turn
            else {
//                LOG << "else (turn)" << endl;
                krembot.Base.drive(0, turning_speed);
            }
            break;
        }

        case State::block: {
//            LOG << "block" << endl;
            // if the robot has food - change state to findBase
            if (hasFood) {
//                LOG << "hasFood(block)" << endl;
                state = State::findBase;
            }
                // if the robot is being pushed while trying to block
            else if (!isFirstBlock && !samePos(blockPos)) {
//                LOG << "!samePos(block)" << endl;
                isFirstBlock = true;
                krembot.Base.drive(100, 0);
                frequencyBlockTimer.start(10000);
                state = State::move;
            }
                // if blockTimer is finished - change state to move (in order not to waste all the time in blocking)
            else if (blockTimer.finished()) {
//                LOG << "blockTimer.finished(block)" << endl;
                isFirstBlock = true;
                krembot.Base.drive(100, 0);
                frequencyBlockTimer.start(10000);
                state = State::move;
            }
                // if the robot doesn't see the opponent base anymore - it means he is on it
            else if (!oppBaseF && !oppBaseR && !oppBaseL) {
//                LOG << "!oppBaseF(block)" << endl;
                krembot.Base.stop();
                // saves the position where the robot stands in order to block
                if (isFirstBlock) {
//                    LOG << "firstTime(block)" << endl;
                    isFirstBlock = false;
                    blockPos = pos;
                }
            }
                // if the robot sees the opponent base in front
            else if (oppBaseF) {
//                LOG << "oppBaseF(block)" << endl;
                krembot.Base.drive(100, 0);
            }
                // if the robot sees the opponent base from right
            else if (oppBaseR) {
//                LOG << "oppBaseR(block)" << endl;
                krembot.Base.drive(30, -1 * turning_speed);
            }
                // if the robot sees the opponent base from left
            else if (oppBaseL) {
//                LOG << "oppBaseL(block)" << endl;
                krembot.Base.drive(30, turning_speed);
            } else {
//                LOG << "else(block)" << endl;
                krembot.Base.drive(100, 0);
            }
            break;
        }

        case State::addHomeBase: {
//            LOG << "addHomeBase" << endl;
            // if the robot has food - change state to findBase
            if (hasFood) {
//                LOG << "hasFood(addHomeBase)" << endl;
                state = State::findBase;
            }
            // if the robot doesn't see our base anymore - it means he is on it or close enough to it
            else if (!baseF && !baseR && !baseL) {
//                LOG << "!baseF(addHomeBase)" << endl;
                // save the current position - and start a timer in order not to add the same area again
                krembot.Base.stop();
                addHomeArea();
                frequencyAddPosTimer.start(add_pos_time);
                state = State::move;
            }
            // if the robot sees our base on front
            else if (baseF) {
//                LOG << "baseF(addHomeBase)" << endl;
                krembot.Base.drive(100, 0);
                state = State::move;
            } else {
//                LOG << "else(addHomeBase)" << endl;
                krembot.Base.drive(100, 0);
            }
            break;
        }

        case State::collision: {
//            LOG << "collision" << endl;
            // if the robot has food - change state to findBase
            if (hasFood) {
//                LOG << "hasFood (collision)" << endl;
                state = State::findBase;
            }
            // if turnTimer is finished - change state to move
            else if (turnTimer.finished()) {
//                LOG << "turnTimer.finished (collision)" << endl;
                state = State::move;
            }
            // turn in order to get out from the collision
            else {
//                LOG << "else (collision)" << endl;
                krembot.Base.drive(0, turning_speed);
            }
            break;
        }

        case State::passRobot: {
//            LOG << "passRobot" << endl;
            // if the robot has food - change state to findBase
            if (hasFood) {
//                LOG << "hasFood (passRobot)" << endl;
                state = State::findBase;
            }
                // if turnTimer is finished - change state to move
            else if (turnTimer.finished()) {
//                LOG << "turnTimer.finished (passRobot)" << endl;
                state = State::move;
            } else {
//                // if there is a robot from left but not from right
                if (colorFR != -1 && colorFL == -1) {
                    direction = 1;
                }
                    // if there is a robot from right but not from left
                else if (colorFL != -1 && colorFR == -1) {
                    direction = -1;
                } else {
                    direction = random_direction();
                }
                // turn
                krembot.Base.drive(50, direction * turning_speed);
            }
            break;
        }
    }
}

/// function that returns true two positions are equal
bool foraging_8_controller::samePos(CVector2 otherPos) {
    if ((pos.GetX() == otherPos.GetX()) && (pos.GetY() == otherPos.GetY())) {
        return true;
    } else {
        return false;
    }
}

/// function that calculates the Angular spd that the robot needs to turn in
int foraging_8_controller::calc_Angular_spd(CDegrees deg) {
    int angularSpd;
    // general case
    if (deg < degreeX.UnsignedNormalize()) {
        angularSpd = -25;
    } else {
        angularSpd = 25;
    }

    // specific cases - to make turn more efficient
    if (degreeX.UnsignedNormalize() > CDegrees(359.50) && (deg <= CDegrees(90) && deg >= CDegrees(0))) {
        angularSpd = 25;
    } else if (degreeX.UnsignedNormalize() < CDegrees(0.5) && (deg <= CDegrees(360) && deg >= CDegrees(270))) {
        angularSpd = -25;
    } else if (degreeX.UnsignedNormalize() > CDegrees(269.5) && (deg <= CDegrees(89.5) && deg >= CDegrees(0))) {
        angularSpd = 25;
    } else if (deg > degreeX.UnsignedNormalize() && (degreeX - deg).UnsignedNormalize().GetValue() < 5) {
        angularSpd = 5;
    } else if (deg < degreeX.UnsignedNormalize() && (degreeX - deg).UnsignedNormalize().GetValue() < 5) {
        angularSpd = -5;
    }
    return angularSpd;
}

/// function that returns true if the robot got to a wanted degree, and false o.w (showed in the tirgul)
bool foraging_8_controller::got_to_orientation(CDegrees degree) {
    Real deg = (degreeX - degree).UnsignedNormalize().GetValue();
    if ((deg > 0.5) && (deg < 359.5)) {
        return false;
    } else {
        return true;
    }
}

/// function that calculates the degree that the robot needs to turn in in order to reach target position
CDegrees foraging_8_controller::calculateDeg(CVector2 target) {
    Real y = target.GetY() - pos.GetY();
    Real x = target.GetX() - pos.GetX();
    CDegrees deg = CDegrees(atan2(y, x) * 180 / M_PI);
    return deg;
}

/// function that returns a position of home base or a position which is close to the home base
CVector2 foraging_8_controller::find_closest_base() {
    vector<CVector2> vec;
    if (!homePos.empty()) {
        vec = homePos;
    } else {
        vec = homeArea;
    }
    CVector2 closestBase;
    float minDis = INFINITY;
    float distance, baseX, baseY, posX, posY;
    // search the closest base
    for (int i = 0; i < vec.size(); i++) {
        baseX = vec[i].GetX();
        baseY = vec[i].GetY();
        posX = pos.GetX();
        posY = pos.GetY();
        distance = sqrt((baseX - posX) * (baseX - posX) + (baseY - posY) * (baseY - posY));
        if (distance < minDis) {
            minDis = distance;
            closestBase = vec[i];
        }
    }
    return closestBase;
}

/// function that returns a random direction from -1 or 1
int foraging_8_controller::random_direction() {
    int r = rand() % 2;
    if (r == 1) {
        return 1;
    }
    return -1;
}

/// function that saves the current position of the robot as a home base position
void foraging_8_controller::addHomePos() {
    if (!count(homePos.begin(), homePos.end(), pos)) {
        homePos.push_back(pos);
    }
}

/// function that saves the current position of the robot as an close area of the home base
void foraging_8_controller::addHomeArea() {
    if (!count(homeArea.begin(), homeArea.end(), pos)) {
        homePos.push_back(pos);
    }
}

/// function that initializes the some of the robot states
void foraging_8_controller::init_environment_states() {
    // if there is a robot from our group in front of us
    if (colorF == homeTeamColor || colorFL == homeTeamColor || colorFR == homeTeamColor) {
        homeBotF = true;
    } else {
        homeBotF = false;
    }
    // if there is a robot from other group in front of us
    if (colorF == opponentTeamColor || colorFL == opponentTeamColor || colorFR == opponentTeamColor) {
        oppBotF = true;
    } else {
        oppBotF = false;
    }
    // if there is a robot (doesn't matter from each group) in front of us
    if (homeBotF || oppBotF) {
        isRobot = true;
    } else {
        isRobot = false;
    }
    // if there is an obstacle in front of us
    if ((_distanceF < dis_threshold || _distanceFL < dis_threshold || _distanceFR < dis_threshold) && !isRobot) {
        isObstcle = true;
    } else {
        isObstcle = false;
    }
    // if our base is in front of us
    if (colorF == homeBaseColor) {
        baseF = true;
    } else {
        baseF = false;
        // if our base is right to us
        if (colorFR == homeBaseColor || colorR == homeBaseColor) {
            baseR = true;
        } else {
            baseR = false;
        }
        // if our base is left to us
        if (colorFL == homeBaseColor || colorL == homeBaseColor) {
            baseL = true;
        } else {
            baseL = false;
        }
    }
    // if opponent base is in front of us
    if (colorF == opponentBaseColor) {
        oppBaseF = true;
    } else {
        oppBaseF = false;
        // if opponent base is right to us
        if (colorFR == opponentBaseColor || colorR == opponentBaseColor) {
            oppBaseR = true;
        } else {
            oppBaseR = false;
        }
        // if our opponent is left to us
        if (colorFL == opponentBaseColor || colorL == opponentBaseColor) {
            oppBaseL = true;
        } else {
            oppBaseL = false;
        }
    }
    // if the robot is in a collision
    if (bumpers.front == BumperState::PRESSED || bumpers.front_left == BumperState::PRESSED ||
        bumpers.front_right == BumperState::PRESSED) {
        isCollision = true;
    } else {
        isCollision = false;
    }
}

/// function that initializes the colors of groups as ints
void foraging_8_controller::init_colors() {
    if (foragingMsg.ourBaseColor == "magenta") {
        homeBaseColor = Color::_magenta;
        opponentBaseColor = Color::_cyan;
        homeTeamColor = Color::_red;
        opponentTeamColor = Color::_green;
    } else {
        homeBaseColor = Color::_cyan;
        opponentBaseColor = Color::_magenta;
        homeTeamColor = Color::_green;
        opponentTeamColor = Color::_red;
    }
}

/// function that converts color into int
int foraging_8_controller::convert_color_to_int(RGBAResult color) {
    if (color.Red == rgb_num && color.Blue == rgb_num && color.Green == 0) {
        return Color::_magenta;
    } else if (color.Red == rgb_num && color.Blue == 0 && color.Green == 0) {
        return Color::_red;
    } else if (color.Green == rgb_num && color.Blue == rgb_num && color.Red == 0) {
        return Color::_cyan;
    } else if (color.Green == rgb_num && color.Blue == 0 && color.Red == 0) {
        return Color::_green;
    } else {
        return -1;
    }
}

/// function that reads colors and uses the robot sensors
void foraging_8_controller::read_colors() {
    // colors
    colorF = convert_color_to_int(krembot.RgbaFront.readRGBA());
    colorFL = convert_color_to_int(krembot.RgbaFrontLeft.readRGBA());
    colorFR = convert_color_to_int(krembot.RgbaFrontRight.readRGBA());
    colorL = convert_color_to_int(krembot.RgbaLeft.readRGBA());
    colorR = convert_color_to_int(krembot.RgbaRight.readRGBA());

    // compute distances
    _distanceF = krembot.RgbaFront.readRGBA().Distance;
    _distanceFL = krembot.RgbaFrontLeft.readRGBA().Distance;
    _distanceFR = krembot.RgbaFrontRight.readRGBA().Distance;

    // bumpers sensors
    bumpers = krembot.Bumpers.read();
}
