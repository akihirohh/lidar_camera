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
	strftime(buf,sizeof(buf), "%Y%m%d_%H%M%S_", &tstruct);
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
	std::string lidar_filename, image_filename;
	std::fstream lidar_file;

	//LIDAR
	int previous_lidar_ts = 0, current_ts = 0;
  std::vector<long> lidar_readings (1080);
	pthread_t thLidar;
	lidarRead::thdata th_lidar;
	th_lidar.b_loop = 0;
  th_lidar.portname = "/dev/ttyACM0";

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
  if(!lidar_file.is_open())
  {
    std::cout << lidar_filename << " lidar_file is not open\n!";
    return -1;
  }

  pthread_create (&thLidar, NULL, &lidarRead::lidarReading, &th_lidar);
  while (!th_lidar.b_loop);

  gettimeofday(&t_start,NULL);

  while(loop)
  {    
    timestamp = currentTime() + std::to_string(millis(t_start));
    try
    {
      th_lidar.mtx.lock();
      std::copy(th_lidar.d.begin(), th_lidar.d.end()-1, lidar_readings.begin());
      current_ts = th_lidar.timestamp;
      th_lidar.mtx.unlock();
    }
    catch (...) { std::cout << "\n\nCouldn't copy latest LiDAR readings...\n\n";}
    
    if(current_ts != previous_lidar_ts)
    {
      cap >> frame;
      std::cout << std::endl << timestamp << "\nLidar samples:\n";
      for (int i = 0; i < lidar_readings.size(); i += lidar_readings.size()/10)
      {
        std::cout << "[" << i << "]: " << lidar_readings[i] << "\t";
      }
      previous_lidar_ts = current_ts;
      lidar_file << timestamp;
      for (int i = 0; i < lidar_readings.size(); i++) //last one is lidar_timestamp
        lidar_file << "|" << lidar_readings[i];
      lidar_file << std::endl;
      imwrite(folder_name + "/" + timestamp + ".jpg", frame);
    }    

    if(kbhit())
    {
      int key = getchar();
      if(key == 'q') loop = 0;
    }
  }
  lidar_file.close();
  popen(("mv " +lidar_filename + " "+ folder_name).c_str(), "w" );
  std::cout << std::endl << lidar_filename << std::endl; 
}
