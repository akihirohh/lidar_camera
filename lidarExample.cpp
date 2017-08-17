#include <iostream>
#include <fstream>
#include <sys/time.h> //gettimeofday
#include "lidarRead.h"
#include "keyboardbreak.h"
#include "opencv2/opencv.hpp"
#include <stdio.h>

//For data logging purpose. It returns a string
//with current time to uniquely name log files
const std::string currentDateTime(){
	time_t		now = time(0);
	struct tm	tstruct;
	char		buf[80];
	tstruct = *localtime(&now);
	strftime(buf,sizeof(buf), "%Y%m%d_%Hh%Mm%Ss",&tstruct);
	return buf;
}
const std::string currentTime(){
	time_t		now = time(0);
	struct tm	tstruct;
	char		buf[80];
	tstruct = *localtime(&now);
	strftime(buf,sizeof(buf), "%Y%m%d,%H%M%S,", &tstruct);
	return buf;
}
int millis(timeval t_start)
{
	struct timeval t;
	gettimeofday(&t,NULL);
	return (t.tv_sec - t_start.tv_sec)*1000 + (t.tv_usec - t_start.tv_usec)/1000;	
}

int main(int argc, char *argv[])
{
	//DATA_LOGGING
	std::string lidar_filename, image_filename, intensity_filename;
	std::fstream lidar_file, intensity_file;

	//LIDAR
	int previous_lidar_ts = 0, current_ts = 0;
  std::vector<long> lidar_distance (1080);
  std::vector<unsigned short> lidar_intensity(1080);
	pthread_t lidar_thread;
	lidarRead::thdata lidar_data;
	lidar_data.b_loop = 0;
  lidar_data.portname = "/dev/ttyACM0";

  //MISC
  int loop = 1, key;
  std::string timestamp;
  struct timeval t_start;

	cv::VideoCapture cap(0);
	cv::Mat frame;
	if(!cap.isOpened())  // check if we succeeded
	{
		std::cout << "Camera not opened!\n";
		return -1;
	}

  std::string folder_name = "Data/" + currentDateTime(), cmd_mkdir = "mkdir " + folder_name;

  popen(cmd_mkdir.c_str(),"r");
  lidar_filename = currentDateTime()+"_lidar_ts.txt";
  lidar_file.open(lidar_filename.c_str(), std::ios_base::out);
  intensity_filename = currentDateTime()+"_intensity_ts.txt";
  intensity_file.open(intensity_filename.c_str(), std::ios_base::out);

  if(!lidar_file.is_open())
  {
    std::cout << lidar_filename << " lidar_file is not open\n!";
    return -1;
  }

  pthread_create (&lidar_thread, NULL, &lidarRead::lidarReading, &lidar_data);
  while (!lidar_data.b_loop);

  gettimeofday(&t_start,NULL);

  while(loop)
  {    
    timestamp = currentTime() + std::to_string(millis(t_start));
    try
    {
      lidar_data.mtx.lock();
      std::copy(lidar_data.distance.begin(), lidar_data.distance.end()-1, lidar_distance.begin());
      std::copy(lidar_data.intensity.begin(), lidar_data.intensity.end(), lidar_intensity.begin());
      current_ts = lidar_data.timestamp;
      lidar_data.mtx.unlock();
    }
    catch (...) { std::cout << "\n\nCouldn't copy latest LiDAR readings...\n\n";}
    
    if(current_ts != previous_lidar_ts)
    {
      cap >> frame;
      std::cout << std::endl << timestamp << "\nLidar samples:\n";
      for (int i = 0; i < lidar_distance.size(); i += lidar_distance.size()/10)
      {
        std::cout << "[" << i << "]: " << lidar_distance[i] << " | "  << lidar_intensity[i] << "\t";;
      }
      previous_lidar_ts = current_ts;
      lidar_file << timestamp;
      for (int i = 0; i < lidar_distance.size(); i++) //last one is lidar_timestamp
        lidar_file << "," << lidar_distance[i];
      lidar_file << std::endl;

      intensity_file << timestamp;
      for (int i = 0; i < lidar_distance.size(); i++) //last one is lidar_timestamp
        intensity_file << "," << lidar_intensity[i];
      intensity_file << std::endl;

      imwrite(folder_name + "/" + timestamp + ".jpg", frame);
    }    

    if(kbhit())
    {
      int key = getchar();
      if(key == 'q') loop = 0;
    }
  }
  lidar_file.close();
  intensity_file.close();
  popen(("mv " +lidar_filename + " "+ folder_name).c_str(), "w" );
  popen(("mv " +intensity_filename + " "+ folder_name).c_str(), "w" );
  std::cout << std::endl << lidar_filename << std::endl; 
}
