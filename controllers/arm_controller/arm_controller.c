#include <webots/robot.h>
#include <webots/keyboard.h>
#include <webots/motor.h>

#include <stdio.h>

#define TIME_STEP 64
#define SPEED 2

int main(int argc, char **argv) 
{
  wb_robot_init();
  
  WbDeviceTag motor1 = wb_robot_get_device("motor1");
  WbDeviceTag motor2 = wb_robot_get_device("motor2");
  
  wb_motor_set_position(motor1, INFINITY);
  wb_motor_set_position(motor2, INFINITY);
  
  wb_motor_set_velocity(motor1, 0.0);   
  wb_motor_set_velocity(motor2, 0.0);
  
  double yawSpeed = 0.0f;
  double pitchSpeed = 0.0f;
  
  wb_keyboard_enable(16);
  
  while (wb_robot_step(TIME_STEP) != -1) 
  {
      yawSpeed = 0.0f;
      pitchSpeed = 0.0f;
      
      char c = wb_keyboard_get_key();
      
      switch(c)
      {
          case 'W':
          yawSpeed = 1.0f;
          break;
          
          case 'S':
          yawSpeed = -1.0f;
          break;
          
          case 'A':
          pitchSpeed = 1.0f;
          break;
          
          case 'D':
          pitchSpeed = -1.0f;
          break;
      }   
      
      wb_motor_set_velocity(motor1, SPEED * pitchSpeed);
      wb_motor_set_velocity(motor2, SPEED * yawSpeed);
  };

  wb_robot_cleanup();

  return 0;
}
