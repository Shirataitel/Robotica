#include "krembot.ino.h"
#include <vector>
#include <algorithm>
#include <cmath>

CVector2 pos;
CDegrees degreeX;
vector<CVector2> homePos;
float _distanceF, _distanceFL, _distanceFR;
int homeBaseColor, opponentBaseColor, homeTeamColor, opponentTeamColor;
int colorF, colorFL, colorFR, colorR, colorL;
bool increaseTime;
bool homeBotF;
bool oppBotF;
bool isObstcle, isRobot;
bool baseF, baseR, baseL, baseFLR;
int _time;

void foraging_8_controller::setup() {
    krembot.setup();
    writeTeamColor();
    teamName = "foraging_8_controller";
    init_colors();
    increaseTime = false;
    direction = 1;
    _time = 300;
    frequencyTurnTimer.start(_time);
}

void foraging_8_controller::loop() {
    krembot.loop();
    pos = posMsg.pos;
    degreeX = posMsg.degreeX;
    int angularSpd;

    if (!isWander && !hasFood) {
        krembot.Base.stop();
        addHomePos();
    }

    read_colors();
    init_environment_states();

    switch (state) {

        case State::move: {
            LOG << "in move" << endl;
            if (hasFood) {
                state = State::findBase;
            } else if (homeBotF) {
                turnTimer.start(200);
                state = State::passRobot;
            } else if (oppBotF) {
                turnTimer.start(200);
                state = State::passRobot;
            } else if (isObstcle) {
                LOG << "isObstcle is on" << endl;
                turnTimer.start(200);
                state = State::turn;
            } else if (frequencyTurnTimer.finished()) {
                /***/
                turnTimer.start(220);
                state = State::turn;
            } else {
                krembot.Base.drive(100, 0);
            }
            break;
        }

        case State::findBase: {
            LOG << "homePos.size()" <<homePos.size() << endl;
            LOG << "in findBase" << endl;
            if (!hasFood) {
                state = State::move;
            } else if (baseF) {
                krembot.Base.drive(100, 0);
            }
            else if (baseR) {
                krembot.Base.drive(30, -100);
            } else if (baseL) {
                krembot.Base.drive(30, 100);
            } else if (isObstcle) {
                if (_distanceFL < _distanceFR) {
                    direction = -1;
                } else {
                    direction = 1;
                }
                state = State::turn;
            } else if (isRobot) {
                if (colorFR != -1 && colorFL == -1) {
                    direction = 1;
                } else if (colorFL != -1 && colorFR == -1) {
                    direction = -1;
                } else {
                    direction = random_direction();
                }
                turnTimer.start(200);
                state = State::turn;
            } else {
                if (homePos.empty()) {
                    krembot.Base.drive(100, 0);
                } else {
                    state = State::turn;
                }
            }
            break;
        }

        case State::turn: {
            LOG << "in turn" << endl;
            if (hasFood && !homePos.empty()) {
                LOG << "if 1" << endl;
                CVector2 closestBase = find_closest_base();
                CDegrees deg = calculateDeg(closestBase);
                if (got_to_orientation(deg)) {
                    krembot.Base.drive(100, 0);
                    state = State::move;
                } else {
                    angularSpd = calc_Angular_spd(deg);
                    krembot.Base.drive(0, angularSpd);
                }
            } else if (isObstcle) {
                krembot.Base.drive(0, 100);
                krembot.Base.drive(0, 100);
                state = State::move;
            } else if (turnTimer.finished()) {
                LOG << "if 2" << endl;
                if (increaseTime) {
                    _time = _time + 400;
                    increaseTime = false;
                } else {
                    increaseTime = true;
                }
                frequencyTurnTimer.start(_time);
                state = State::move;
            } else {
                LOG << "if 3" << endl;
                krembot.Base.drive(0, 100);
            }
            break;
        }

        case State::passRobot: {
            LOG << "in passRobot" << endl;
            if (hasFood) {
                state = State::findBase;
            } else if (turnTimer.finished()) {
                state = State::move;
            } else {
                krembot.Base.drive(50, direction * 100);
            }
            break;
        }
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
//    if (degreeX.UnsignedNormalize() > CDegrees(359.50) && (deg <= CDegrees(90) && deg >= CDegrees(0))) {
//        angularSpd = 25;
//    } else if (degreeX.UnsignedNormalize() < CDegrees(0.5) && deg == downDeg) {
//        angularSpd = -25;
//    } else if (degreeX.UnsignedNormalize() > CDegrees(269.5) && deg == rightDeg) {
//        angularSpd = 25;
//    }
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
    CVector2 closestBase;
    float minDis = INFINITY;
    float distance, baseX, baseY, posX, posY;
    for (int i = 0; i < homePos.size(); i++) {
        baseX = homePos[i].GetX();
        baseY = homePos[i].GetY();
        posX = pos.GetX();
        posY = pos.GetY();
        distance = sqrt((baseX - posX) * (baseX - posX) + (baseY - posY) * (baseY - posY));
        if (distance < minDis) {
            minDis = distance;
            closestBase = homePos[i];
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
    LOG << "in addHomePos" << endl;
    if (!count(homePos.begin(), homePos.end(), pos)) {
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
    if (_distanceF < 5 && !isRobot) {
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
    if (baseF || baseL || baseR) {
        baseFLR = true;
    } else {
        baseFLR = false;
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
    if (color.Red == 255) {
        if (color.Blue == 225) {
            return Color::_magenta;
        } else {
            return Color::_red;
        }
    } else if (color.Green == 255) {
        if (color.Blue == 225) {
            return Color::_cyan;
        } else {
            return Color::_green;
        }
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
}
