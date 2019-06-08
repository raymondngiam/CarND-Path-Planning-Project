#pragma once
#include <time.h>
#include <fstream>
#include <vector>

using namespace std;

class Tools {
public:
  /**
  * Constructor.
  */
  Tools();

  /**
  * Destructor.
  */
  virtual ~Tools();

  ofstream f_log_sensor;
	ofstream f_log_path;

  void LogSensor(std::vector<std::vector<double>> sensor_fusion);
	void LogPath(int prev_size, double x, double y, std::vector<double> pathX, std::vector<double> pathY);
	std::string GenTimeStamp();
};


