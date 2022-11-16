//Shira Taitelbaum 322207341
#include <Krembot/controller/krembot_controller.h>

class ex1_controller : public KrembotController {
private:
    bool isFirst = true;
public:
    ParticleObserver Particle;
    ~ex1_controller() = default;
    void setup();
    void loop();

    void Init(TConfigurationNode &t_node) override {
        KrembotController::Init(t_node);
        if ( ! krembot.isInitialized() ) {
            throw std::runtime_error("krembot.ino.cpp: krembot wasn't initialized in controller");
        }
        Particle.setName(krembot.getName());
    }
    void ControlStep() override {
        if(isFirst) {
            setup();
            isFirst = false;
        }
        loop();
    }
};


REGISTER_CONTROLLER(ex1_controller, "ex1_controller")
