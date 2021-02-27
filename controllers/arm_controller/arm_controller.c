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
  
  double leftSpeed = 0.0f;
  double rightSpeed = 0.0f;
  
  wb_keyboard_enable(16);
  
  while (wb_robot_step(TIME_STEP) != -1) 
  {
      leftSpeed = 0.0f;
      rightSpeed = 0.0f;
      
      char c = wb_keyboard_get_key();
      
      switch(c)
      {
          case 'W':
          leftSpeed = -1.0f;
          break;
          
          case 'S':
          leftSpeed = 1.0f;
          break;
          
          case 'A':
          rightSpeed = -1.0f;
          break;
          
          case 'D':
          rightSpeed = -1.0f;
          break;
      }   
      
      wb_motor_set_velocity(motor1, SPEED * rightSpeed);
      wb_motor_set_velocity(motor2, SPEED * leftSpeed);
  };

  wb_robot_cleanup();

  return 0;
}
