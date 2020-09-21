#include "ODriveTool.h"
#define PI 3.14159262

// Printing with stream operator
template<class T> inline Print& operator <<(Print &obj,     T arg) { obj.print(arg);    return obj; }
template<>        inline Print& operator <<(Print &obj, float arg) { obj.print(arg, 4); return obj; }

ODriveTool::ODriveTool(HardwareSerial& serial)
    : odrive_(serial), ODriveArduino(serial)
{
    odrive_reboot();
}

void ODriveTool::odrive_reboot()
{
    odrive_.begin(115200);
    odrive_ << "sr\n";
    delay(3000);
}

void ODriveTool::odrive_init(int axis, float vel_lim, float current_lim)
{
    odrive_.begin(115200);
    int requested_state;
    char buf[50];
    odrive_ << "w axis" << axis << ".controller.config.vel_limit " << vel_lim << '\n';
    odrive_ << "w axis" << axis << ".motor.config.current_lim " << current_lim << '\n';
    requested_state = ODriveArduino::AXIS_STATE_MOTOR_CALIBRATION;
    ODriveArduino::run_state(axis, requested_state, true);
    requested_state = ODriveArduino::AXIS_STATE_ENCODER_OFFSET_CALIBRATION;
    ODriveArduino::run_state(axis, requested_state, true);
    requested_state = ODriveArduino::AXIS_STATE_CLOSED_LOOP_CONTROL;
    ODriveArduino::run_state(axis, requested_state, false);
}

float ODriveTool::get_velocity(int axis, float reduction_ratio, int encoder_cpr){
    float encoder_vel=0, velocity=0;
    odrive_ << "r axis" << axis << ".encoder.vel_estimate\n";
    encoder_vel=ODriveArduino::readFloat();
    return velocity=(encoder_vel*(PI/encoder_cpr))/reduction_ratio ;
}

float ODriveTool::get_position(int axis, float reduction_ratio, int encoder_cpr){
    float encoder_pos=0, position=0;
    odrive_ << "r axis" << axis << ".encoder.pos_estimate\n";
    encoder_pos=ODriveArduino::readFloat();
    return position=((360*(fmod(encoder_pos,360)))/encoder_cpr)/reduction_ratio ;
}

float ODriveTool::get_effort(int axis, float reduction_ratio, int kv){
    float encoder_current=0, effort=0;
    odrive_ << "r axis" << axis << ".motor.current_control.Iq_setpoint\n";
    encoder_current=ODriveArduino::readFloat();
    return effort=((8.27*encoder_current)/kv)*reduction_ratio ;
}

float ODriveTool::get_voltage(){
    float voltage=0;
    odrive_ << "r vbus_voltage\n";
    voltage=ODriveArduino::readFloat();
    return voltage;
}
