#include <webots/robot.h>
#include <webots/camera.h>
#include <webots/lidar.h>

#include <stdio.h>

#define TIME_STEP 64

int main(int argc, char **argv) 
{
	wb_robot_init();
	WbDeviceTag camera = wb_robot_get_device("camera");

	wb_camera_enable(camera, TIME_STEP);
	
	WbDeviceTag lidar = wb_robot_get_device("lidar");
	wb_lidar_enable(lidar, TIME_STEP);
	wb_lidar_enable_point_cloud(lidar);

	while (wb_robot_step(TIME_STEP) != -1) 
	{
		int count = wb_lidar_get_number_of_points(lidar);
		
		WbLidarPoint *points = wb_lidar_get_point_cloud(lidar);

		FILE* file = 0;
		fopen_s(&file, "E:\\Development\\OpenGL\\bin\\point.csv", "w");

		if (file)
		{
			for(int n = 0; n < count; n++)
			{
				fprintf(file, "%f,%f,%f\n", points[n].x, points[n].y, points[n].z);
			}
			fclose(file);
		}

		//printf("point count: %d\n", count);
	};
	
	wb_robot_cleanup();
	
	return 0;
}
