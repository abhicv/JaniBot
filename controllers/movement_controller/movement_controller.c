#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/keyboard.h>

#define TIME_STEP 64
#define SPEED 10

int main(int argc, char **argv) 
{
   wb_robot_init();
  
   //front wheels
   WbDeviceTag wheel_1 = wb_robot_get_device("wheel_1");
   WbDeviceTag wheel_3 = wb_robot_get_device("wheel_3");
   
   //rear wheels
   WbDeviceTag wheel_2 = wb_robot_get_device("wheel_2");
   WbDeviceTag wheel_4 = wb_robot_get_device("wheel_4");
   
   // set the target position of the motors
   wb_motor_set_position(wheel_1, INFINITY);
   wb_motor_set_position(wheel_3, INFINITY);
   wb_motor_set_position(wheel_2, INFINITY);
   wb_motor_set_position(wheel_4, INFINITY);
   
   wb_motor_set_velocity(wheel_1, 0.0);   
   wb_motor_set_velocity(wheel_3, 0.0);
   wb_motor_set_velocity(wheel_2, 0.0);
   wb_motor_set_velocity(wheel_4, 0.0);
   
   wb_keyboard_enable(16);
   
   double leftSpeed = 1.0f;
   double rightSpeed = 1.0f;
      
   while (wb_robot_step(TIME_STEP) != -1) 
   {
      leftSpeed = 0.0f;
      rightSpeed = 0.0f;
      
      switch(wb_keyboard_get_key())
      {
          case WB_KEYBOARD_LEFT:
          leftSpeed = -1.0f;
          rightSpeed = 1.0f;
          break;
          
          case WB_KEYBOARD_UP:
          leftSpeed = 1.0f;
          rightSpeed = 1.0f;
          break;
          
          case WB_KEYBOARD_RIGHT:
          leftSpeed = 1.0f;
          rightSpeed = -1.0f;
          break;
          
          case WB_KEYBOARD_DOWN:
          leftSpeed = -1.0f;
          rightSpeed = -1.0f;
          break;
      }   
      
      wb_motor_set_velocity(wheel_1, SPEED * leftSpeed);
      wb_motor_set_velocity(wheel_3, SPEED * rightSpeed);
      wb_motor_set_velocity(wheel_2, SPEED * leftSpeed);
      wb_motor_set_velocity(wheel_4, SPEED * rightSpeed);  
   }

   wb_robot_cleanup();

  return 0;
}
