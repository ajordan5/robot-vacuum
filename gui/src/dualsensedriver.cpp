#include "dualsensedriver.h"

DualSenseDriver::DualSenseDriver()
{
    init_controller();
}

DualSenseDriver::~DualSenseDriver()
{
    DS5W::freeDeviceContext(&context);
}

void DualSenseDriver::init_controller()
{
    unsigned int controllersCount{0};

    switch(DS5W::enumDevices(infos, 16, &controllersCount)){
        case DS5W_OK:
            connected = true;
        case DS5W_E_INSUFFICIENT_BUFFER:
            connected = true;
            break;
        default:
            connected = false;
            break;
    }

    initialized = !DS5W_FAILED(DS5W::initDeviceContext(&infos[0], &context));

}

VacuumControlState DualSenseDriver::get_control()
{
    DS5W::DS5InputState inState;
    VacuumControlState controller;

    if (is_available() && DS5W_SUCCESS(DS5W::getDeviceInputState(&context, &inState)))
    {

        DS5W::DS5OutputState outState;
        ZeroMemory(&outState, sizeof(DS5W::DS5OutputState));

        double forwardBack = normalize_analog_stick_int((int)inState.leftStick.y);
        double leftRight = normalize_analog_stick_int((int)inState.leftStick.x);
        double rightTrigger = normalize_trigger_uint(inState.rightTrigger);


        controller.drive = forwardBack;
        controller.turn = leftRight;
        controller.turbo = rightTrigger;
        set_vibration(inState, outState, forwardBack);
        set_haptic_feedback(outState);

        DS5W::setDeviceOutputState(&context, &outState);

    }

    return controller;
}

void DualSenseDriver::set_vibration(const DS5W::DS5InputState& inState, DS5W::DS5OutputState& outState, double forwardBack)
{
    if (inState.leftTrigger == 255)
        enableRumble = !enableRumble;
    if (enableRumble)
        outState.rightRumble =  abs((int)(forwardBack*5));
}

void DualSenseDriver::set_haptic_feedback(DS5W::DS5OutputState& outState)
{
    DS5W::TriggerEffect trigger;
    trigger.effectType = DS5W::TriggerEffectType::ContinuousResitance;
    trigger.Continuous.startPosition = 0;
    trigger.Continuous.force = 120;
    outState.rightTriggerEffect = trigger;

}

double normalize_analog_stick_int(int input)
{
    if(is_analog_stick_dead_zone(input)) return 0;

    return (double)input/128;
}

double normalize_trigger_uint(int input)
{

    return (double)input/255;
}

bool is_analog_stick_dead_zone(int input)
{
    return abs(input) < 6;
}
