#include <iostream>
#include "tools.h"

Tools::Tools() {
	f_log_sensor.open("log_sensor.csv", std::ios_base::out);
	f_log_sensor<<"time,id,x,y,vx,vy,s,d"<<std::endl;
	f_log_path.open("log_path.csv", std::ios_base::out);
	f_log_path<<"time,prevSize,x,y,pathX,pathY"<<std::endl;
}

Tools::~Tools() {}

void Tools::LogSensor(std::vector<std::vector<double>> sensor_fusion){
	std::string time_string = GenTimeStamp();
  for(auto& item:sensor_fusion){	f_log_sensor<<time_string<<","<<item[0]<<","<<item[1]<<","<<item[2]<<","<<item[3]<<","<<item[4]<<","<<item[5]<<","<<item[6]<<std::endl;
	}
}

void Tools::LogPath(int prev_size, double x, double y, std::vector<double> pathX, std::vector<double> pathY){
	std::string time_string = GenTimeStamp();
	std::string xs = "";
	std::string ys = "";
  for(int i=0; i<pathX.size(); i++){
		if(i==0)
		{
			xs=std::to_string(pathX[i]);
			ys=std::to_string(pathY[i]);
		}
		else
		{
			xs+= ":"+std::to_string(pathX[i]);
			ys+= ":"+std::to_string(pathY[i]);
		}
	}
	f_log_path<<time_string<<","<<prev_size<<","<<x<<","<<y<<","<<xs<<","<<ys<<std::endl;
}

std::string Tools::GenTimeStamp(){
	struct timespec now;
	timespec_get(&now, TIME_UTC);
	char buff1[50];
	sprintf (buff1, ".%09ld", now.tv_nsec);
	std::string nano_sec_string(buff1);
	char buff2[100];
	strftime(buff2, sizeof buff2, "%D %T", gmtime(&now.tv_sec));
	std::string time_string(buff2);
	time_string = time_string + nano_sec_string;
	return time_string;
}

