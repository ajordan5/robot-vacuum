#ifndef DUALSENSEDRIVER_H
#define DUALSENSEDRIVER_H

#include "ds5w.h"
#include <Windows.h>

struct VacuumControlState
{
    double drive;
    double turn;
    double turbo;
};

class DualSenseDriver
{
public:
    DualSenseDriver();
    ~DualSenseDriver();
    VacuumControlState get_control();
    bool is_available() {return (connected && initialized);}

private:
    void init_controller();
    void set_haptic_feedback(DS5W::DS5OutputState& outState);
    void set_vibration(const DS5W::DS5InputState& inState, DS5W::DS5OutputState& outState, double forwardBack);
    bool connected;
    bool initialized;
    bool enableRumble{false};
    DS5W::DeviceEnumInfo infos[16];
    DS5W::DeviceContext context;
};

double normalize_analog_stick_int(int input);
double normalize_trigger_uint(int input);
bool is_analog_stick_dead_zone(int input);

#endif // DUALSENSEDRIVER_H
