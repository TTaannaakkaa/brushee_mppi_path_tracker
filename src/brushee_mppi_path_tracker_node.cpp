//  Copyright 2025 amsl

#include "brushee_mppi_path_tracker/brushee_mppi_path_tracker.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "brushee_mppi_path_tracker");
  BrusheeMppiPathTracker brushee_mppi_path_tracker;
  brushee_mppi_path_tracker.process();
  return 0;
}
