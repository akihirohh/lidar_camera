#include "lidarRead.h"

namespace lidarRead
{	
	bool isLidarOpen(Urg_driver& urg)
	{
		return urg.is_open();
	}
	void closeLidar(Urg_driver& urg)
	{
		urg.close();
	}
	int initLidar(int argc, const char **argv, Urg_driver& urg)
	{
			Connection_information information(argc, argv);
			do
			{
					if (!urg.open(information.device_or_ip_name(), information.baudrate_or_port_number(), information.connection_type()))
					{
							std::cout << "Urg_driver::open(): " << information.device_or_ip_name() << ": " << urg.what() << std::endl;
					} 
					if (urg.max_data_size()<0) urg.close();
			}while(urg.max_data_size()<0);

	#if 1
			urg.set_scanning_parameter(urg.deg2step(-135), urg.deg2step(+135), 0);
	#endif
			enum { Capture_times = 1 };
			urg.start_measurement(Urg_driver::Distance, Urg_driver::Infinity_times, 0);
	}

	std::vector <long> getLidar(Urg_driver& urg)
	{
			long time_stamp = 0;
			std::vector<long> data;
			int success = 0;
			success = urg.get_distance(data, &time_stamp);
			if(!success)
			{
					std::cout << "Urg_driver::get_distance(): " << urg.what() << std::endl;
					data.push_back(-1);
			}     
			else
			{
					data.push_back(time_stamp);
			}
			return data;
	}
	void *lidarReading( void *ptr )
	{
		Urg_driver urg0;
		thdata *data;
		data = (thdata *) ptr;
		int b_loop = 1;	
		std::vector<long> readings;

		data->mtx.lock();
		char const *arr0[] = { "lidar", "-s", data->portname.c_str() };	
		initLidar(3, arr0, urg0);
		data->d = getLidar(urg0);
		data->b_loop = 1;
		data->mtx.unlock();

		while(b_loop)
		{
			try
			{
				readings = getLidar(urg0); 				
				if (readings.back()==-1)
				{
					std::cout << "ERROR";
					closeLidar(urg0);
					initLidar(3,arr0,urg0);
					readings = getLidar(urg0);
				}  
			}
			catch (...)
			{
				if (urg0.max_data_size() < 0) 
				{
					std::cout << "\n\n\n!!!ERROR LIDAR!!!\n\n\n";
					b_loop = 0;					
				}      
			}  				
			data->mtx.lock();
			std::copy(readings.begin(), readings.end()-1, data->d.begin());
			data->timestamp = readings.back();	
			if(b_loop) b_loop = data->b_loop;
			else data->b_loop = b_loop;
			data->mtx.unlock();
		}
		closeLidar(urg0);
	}	
}
