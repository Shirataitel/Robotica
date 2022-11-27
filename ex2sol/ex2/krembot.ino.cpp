//shira taitelbaum 322207341
#include "krembot.ino.h"

enum State {
    move,
    turn
};

State state;
SandTimer sandTimer;
int dir;
int timer = 100;

void ex2_controller::setup() {
    krembot.setup();
    krembot.Led.write(0, 255, 0);
}

void ex2_controller::loop() {
    krembot.loop();
    switch (state) {
        case State::move: {
            if (sandTimer.finished()) {
                state = State::turn;
                timer = (rand() % 1001);
                sandTimer.start(timer);
                krembot.Led.write(255, 0, 0);
            } else {
                float distance = krembot.RgbaFront.readRGBA().Distance;
                if (distance < 15) {
                    state = State::turn;
                    timer = (rand() % 1001);
                    sandTimer.start(timer);
                    krembot.Led.write(255, 0, 0);
                } else {
                    krembot.Base.drive(100, 0);
                }
            }
            break;
        }
        case State::turn: {
            dir = (rand() % 201) - 100;
            krembot.Base.drive(0, dir);
            state = State::move;
            krembot.Led.write(0, 255, 0);
            break;
        }
    }

}
