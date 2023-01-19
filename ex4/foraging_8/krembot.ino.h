#include <Krembot/controller/krembot_controller.h>
#include "controllers/foraging/krembot.ino.h"


class foraging_8_controller : public foraging_controller {
private:
    enum State{
        turn,
        move,
        findBase,
        block,
        addHomeBase,
        passRobot
    };

    enum Color{
        _red,
        _green,
        _cyan,
        _magenta
    };

    State state = move;
    SandTimer turnTimer, frequencyTurnTimer, blockTimer, frequencyBlockTimer, frequencyAddPosTimer, startCover;
    int turning_speed = 100;
    int direction = 1;
    Colors ourColor, opponentColor;
public:
    void setup() override;
    void loop() override;

    void addHomePos();
    void addHomeArea();
    void init_colors();
    int convert_color_to_int(RGBAResult color);
    void read_colors();
    void init_environment_states();
    int random_direction();
    CVector2 find_closest_base();
    CDegrees calculateDeg(CVector2 target);
    bool got_to_orientation(CDegrees degree);
    int calc_Angular_spd(CDegrees deg);

};


REGISTER_CONTROLLER(foraging_8_controller, "foraging_8_controller")