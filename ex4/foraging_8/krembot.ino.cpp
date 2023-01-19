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

    read_colors();
    init_environment_states();

    if (startCover.finished()) {
        _time = start_time;
        frequencyTurnTimer.start(_time);
        startCover.start(1000000);
    }

    switch (state) {

        case State::move: {
//            LOG << "move" << endl;
            if (hasFood) {
                state = State::findBase;
            } else if (oppBaseF && frequencyBlockTimer.finished()) {
//                LOG << "oppBaseF(move)" << endl;
                krembot.Base.drive(100, 0);
                blockTimer.start(block_time);
                state = State::block;
            } else if (oppBaseR && frequencyBlockTimer.finished()) {
//                LOG << "oppBaseR(move)" << endl;
                krembot.Base.drive(30, -1 * turning_speed);
                blockTimer.start(block_time);
                state = State::block;
            } else if (oppBaseL && frequencyBlockTimer.finished()) {
//                LOG << "oppBaseL(move)" << endl;
                krembot.Base.drive(30, turning_speed);
                blockTimer.start(block_time);
                state = State::block;
            } else if (baseF && frequencyAddPosTimer.finished()) {
//                LOG << "baseF(move)" << endl;
                krembot.Base.drive(100, 0);
                state = State::addHomeBase;
            } else if (baseR && frequencyAddPosTimer.finished()) {
//                LOG << "baseR(move)" << endl;
                krembot.Base.drive(30, -1 * turning_speed);
                state = State::addHomeBase;
            } else if (baseL && frequencyAddPosTimer.finished()) {
//                LOG << "baseL(move)" << endl;
                krembot.Base.drive(30, turning_speed);
                state = State::addHomeBase;
            } else if (isCollision) {
//                LOG << "bumped" << endl;
                turnTimer.start(440);
                state = State::collision;
            } else if (isRobot) {
//                LOG << "isRobot (move)" << endl;
                turnTimer.start(200);
                state = State::passRobot;
            } else if (isObstcle) {
//                LOG << "obstcle (move)" << endl;
                turnTimer.start(200);
                state = State::turn;
            } else if (frequencyTurnTimer.finished()) {
//                LOG << "frequencyTurnTimer.finished() (move)" << endl;
                turnTimer.start(220);
                state = State::turn;
            } else {
//                LOG << "else (move)" << endl;
                krembot.Base.drive(100, 0);
            }
            break;
        }

        case State::findBase: {
//            LOG << "findBase" << endl;
            if (!hasFood) {
//                LOG << "not hasFood(findBase)" << endl;
                krembot.Base.stop();
                addHomePos();
                frequencyAddPosTimer.start(add_pos_time);
                state = State::move;
                startCover.start(5000);
            } else if (baseF) {
//                LOG << "baseF(findBase)" << endl;
                krembot.Base.drive(100, 0);
            } else if (baseR) {
//                LOG << "baseR(findBase)" << endl;
                krembot.Base.drive(30, -1 * turning_speed);
            } else if (baseL) {
//                LOG << "baseL(findBase)" << endl;
                krembot.Base.drive(30, turning_speed);
            } else if (isObstcle) {
//                LOG << "isObstcle(findBase)" << endl;
                if (_distanceFL < _distanceFR) {
                    direction = -1;
                } else {
                    direction = 1;
                }
                krembot.Base.drive(50, direction * turning_speed);
            } else if (isRobot) {
                krembot.Base.drive(100, 0);
            } else {
//                LOG << "else(findBase)" << endl;
                if (homePos.empty() && homeArea.empty()) {
//                    LOG << "homePos & homeArea empty(findBase)" << endl;
                    krembot.Base.drive(100, 0);
                } else {
//                    LOG << "else else(findBase)" << endl;
                    turnTimer.start(200);
                    state = State::turn;
                }
            }
            break;
        }

        case State::turn: {
//            LOG << "turn" << endl;
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
            }else if (isObstcle) {
//                LOG << "isObstcle(turn)" << endl;
                krembot.Base.drive(0, turning_speed);
                state = State::move;
            } else if (turnTimer.finished()) {
//                LOG << "turnTimer.finished (turn)" << endl;
                if (increaseTime) {
                    _time = _time + 400;
                    increaseTime = false;
                } else {
                    increaseTime = true;
                }
                frequencyTurnTimer.start(_time);
                state = State::move;
            } else {
//                LOG << "else (turn)" << endl;
                krembot.Base.drive(0, turning_speed);
            }
            break;
        }

        case State::block: {
//            LOG << "block" << endl;
            if (hasFood) {
//                LOG << "hasFood(block)" << endl;
                state = State::findBase;
            } else if (!isFirstBlock && !samePos(blockPos)) {
//                LOG << "!samePos(block)" << endl;
                isFirstBlock = true;
                krembot.Base.drive(100, 0);
                frequencyBlockTimer.start(10000);
                state = State::move;
            } else if (blockTimer.finished()) {
//                LOG << "blockTimer.finished(block)" << endl;
                isFirstBlock = true;
                krembot.Base.drive(100, 0);
                frequencyBlockTimer.start(10000);
                state = State::move;
            } else if (!oppBaseF && !oppBaseR && !oppBaseL) {
//                LOG << "!oppBaseF(block)" << endl;
                krembot.Base.stop();
                if (isFirstBlock) {
//                    LOG << "firstTime(block)" << endl;
                    isFirstBlock = false;
                    blockPos = pos;
                }
            } else if (oppBaseF) {
//                LOG << "oppBaseF(block)" << endl;
                krembot.Base.drive(100, 0);
            } else if (oppBaseR) {
//                LOG << "oppBaseR(block)" << endl;
                krembot.Base.drive(30, -1 * turning_speed);
            } else if (oppBaseL) {
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
            if (hasFood) {
//                LOG << "hasFood(addHomeBase)" << endl;
                state = State::findBase;
            } else if (!baseF && !baseR && !baseL) {
//                LOG << "!baseF(addHomeBase)" << endl;
                krembot.Base.stop();
                addHomeArea();
                frequencyAddPosTimer.start(add_pos_time);
                state = State::move;
            } else if (baseF) {
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
            if (hasFood) {
//                LOG << "hasFood (collision)" << endl;
                state = State::findBase;
            } else if (turnTimer.finished()) {
//                LOG << "turnTimer.finished (collision)" << endl;
                state = State::move;
            } else {
//                LOG << "else (collision)" << endl;
                krembot.Base.drive(0, turning_speed);
            }
            break;
        }

        case State::passRobot: {
//            LOG << "passRobot" << endl;
            if (hasFood) {
//                LOG << "hasFood (passRobot)" << endl;
                state = State::findBase;
            } else if (turnTimer.finished()) {
//                LOG << "turnTimer.finished (passRobot)" << endl;
                state = State::move;
            } else {
//                LOG << "else (passRobot)" << endl;
                if (colorFR != -1 && colorFL == -1) {
                    direction = 1;
                } else if (colorFL != -1 && colorFR == -1) {
                    direction = -1;
                } else {
                    direction = random_direction();
                }
                krembot.Base.drive(50, direction * turning_speed);
            }
            break;
        }
    }
}

/// function that returns true if the robot got to a wanted cell, and false o.w (showed in the tirgul)
bool foraging_8_controller::samePos(CVector2 otherPos) {
    if ((pos.GetX() == otherPos.GetX()) && (pos.GetY() == otherPos.GetY())) {
        return true;
    } else {
        return false;
    }
}

/// function that calculates the Angular spd that the robot needs to turn
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

CDegrees foraging_8_controller::calculateDeg(CVector2 target) {
    Real y = target.GetY() - pos.GetY();
    Real x = target.GetX() - pos.GetX();
    CDegrees deg = CDegrees(atan2(y, x) * 180 / M_PI);
    return deg;
}

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

int foraging_8_controller::random_direction() {
    int r = rand() % 2;
    if (r == 1) {
        return 1;
    }
    return -1;
}

void foraging_8_controller::addHomePos() {
    if (!count(homePos.begin(), homePos.end(), pos)) {
        homePos.push_back(pos);
    }
}

void foraging_8_controller::addHomeArea() {
    if (!count(homeArea.begin(), homeArea.end(), pos)) {
        homePos.push_back(pos);
    }
}

void foraging_8_controller::init_environment_states() {
    if (colorF == homeTeamColor || colorFL == homeTeamColor || colorFR == homeTeamColor) {
        homeBotF = true;
    } else {
        homeBotF = false;
    }
    if (colorF == opponentTeamColor || colorFL == opponentTeamColor || colorFR == opponentTeamColor) {
        oppBotF = true;
    } else {
        oppBotF = false;
    }
    if (homeBotF || oppBotF) {
        isRobot = true;
    } else {
        isRobot = false;
    }
    if ((_distanceF < dis_threshold || _distanceFL < dis_threshold || _distanceFR < dis_threshold) && !isRobot) {
        isObstcle = true;
    } else {
        isObstcle = false;
    }
    if (colorF == homeBaseColor) {
        baseF = true;
    } else {
        baseF = false;
        if (colorFR == homeBaseColor || colorR == homeBaseColor) {
            baseR = true;
        } else {
            baseR = false;
        }
        if (colorFL == homeBaseColor || colorL == homeBaseColor) {
            baseL = true;
        } else {
            baseL = false;
        }
    }
    if (colorF == opponentBaseColor) {
        oppBaseF = true;
    } else {
        oppBaseF = false;
        if (colorFR == opponentBaseColor || colorR == opponentBaseColor) {
            oppBaseR = true;
        } else {
            oppBaseR = false;
        }
        if (colorFL == opponentBaseColor || colorL == opponentBaseColor) {
            oppBaseL = true;
        } else {
            oppBaseL = false;
        }
    }
    if (bumpers.front == BumperState::PRESSED || bumpers.front_left == BumperState::PRESSED ||
        bumpers.front_right == BumperState::PRESSED) {
        isCollision = true;
    } else {
        isCollision = false;
    }
}

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

void foraging_8_controller::read_colors() {
    colorF = convert_color_to_int(krembot.RgbaFront.readRGBA());
    colorFL = convert_color_to_int(krembot.RgbaFrontLeft.readRGBA());
    colorFR = convert_color_to_int(krembot.RgbaFrontRight.readRGBA());
    colorL = convert_color_to_int(krembot.RgbaLeft.readRGBA());
    colorR = convert_color_to_int(krembot.RgbaRight.readRGBA());

    _distanceF = krembot.RgbaFront.readRGBA().Distance;
    _distanceFL = krembot.RgbaFrontLeft.readRGBA().Distance;
    _distanceFR = krembot.RgbaFrontRight.readRGBA().Distance;

    bumpers = krembot.Bumpers.read();
}
