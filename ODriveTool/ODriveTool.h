#ifndef ODRIVE_TOOL_H
#define ODRIVE_TOOL_H

#include <SoftwareSerial.h>
#include <ODriveArduino.h>

class ODriveTool:public ODriveArduino{
    public:
        ODriveTool(HardwareSerial& serial);
        void odrive_init(int axis, float vel_lim = 20000 , float current_lim = 50.0);
        void odrive_reboot();
        float get_velocity(int axis,float reduction_ratio=1 , int encoder_cpr=4000);
        float get_position(int axis, float reduction_ratio=1, int encoder_cpr=4000);
        float get_effort(int axis, float reduction_ratio =1, int KV =100);        
        float get_voltage();  
    private:
        HardwareSerial& odrive_;
};

#endif
