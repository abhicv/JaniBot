#include <webots/robot.h>
#include <webots/keyboard.h>
#include <webots/motor.h>

#include <stdio.h>

#define TIME_STEP 64
#define SPEED 2

int main(int argc, char **argv) 
{
  wb_robot_init();
  
  WbDeviceTag baseMotor = wb_robot_get_device("base_motor");
  
  double pitchSpeed = 0.0f;
  
  wb_keyboard_enable(16);
  
  while (wb_robot_step(TIME_STEP) != -1) 
  {
      pitchSpeed = 0.0f;
      
      char c = wb_keyboard_get_key();
      
      switch(c)
      {
          case 'A':
          pitchSpeed = 1.0f;
          break;
      }   
      
      wb_motor_set_velocity(baseMotor, SPEED * pitchSpeed);
  };

  wb_robot_cleanup();

  return 0;
}
