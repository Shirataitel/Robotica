//Shira Taitelbaum 322207341
#include "krembot.ino.h"

void ex1_controller::setup() {
    krembot.setup();
    /*
     * todo: Your setup code goes here
     */
    krembot.Led.write(0,255,0);
}

void ex1_controller::loop() {
    krembot.loop();
    /*
     * todo: Your loop code goes here
     */
    float distance = krembot.RgbaFront.readRGBA().Distance;
    if (distance < 15)
    {
        krembot.Base.stop();
        krembot.Led.write(255,0,0);
    } else
    {
        krembot.Base.drive(100,0);
    }
}

