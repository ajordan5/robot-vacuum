#ifndef DUALSENSEDRIVER_H
#define DUALSENSEDRIVER_H

#include "ds5w.h"
#include <Windows.h>
#include <iostream>

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
    bool connected;
    bool initialized;
    DS5W::DeviceEnumInfo infos[16];
    DS5W::DeviceContext context;
};

double normalize_int(int input);
double normalize_uint(int input);
bool is_dead_zone(int input);

#endif // DUALSENSEDRIVER_H