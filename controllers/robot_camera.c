#include <webots/robot.h>
#include <webots/camera.h>

#define TIME_STEP 64

int main(int argc, char **argv) {

  wb_robot_init();

  WbDeviceTag camera = wb_robot_get_device("camera");
  wb_camera_enable(camera, TIME_STEP); 
  
  while (wb_robot_step(TIME_STEP) != -1) {

  };

  wb_robot_cleanup();

  return 0;
}
