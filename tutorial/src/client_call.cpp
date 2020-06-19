#include <ros/ros.h>
#include <tutorial/my_service.h>
#include <string>
#include <ctime>

// Get current time with format YYYY-MM-DD-HH-mm-ss
std::string currentDataTime(){
  time_t now = time(0);
  struct tm tstruct;
  char buf[80];
  tstruct = *localtime(&now);
  strftime(buf, sizeof(buf), "%Y-%m-%d-%H-%M-%S", &tstruct);
  return buf;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "call_from_client_node");
  ros::NodeHandle nh;
  ros::ServiceClient client = nh.serviceClient<tutorial::my_service>\
				("my_service");
  tutorial::my_service srv;
  std::string filename = "test";
  int seq = 0, tried = 0;
  // Cost 10 seconds to record
  while (seq < 10){
    srv.request.filename = filename + "_" + currentDataTime();
    if(client.call(srv)){ 
      ROS_INFO_STREAM(srv.response.result);
      ros::Duration(1.0).sleep();
      seq++;
    }
    else {
      if(tried>=3) return -1; // Exit if we tried three times
      ROS_ERROR("Failed to call the service, wait 3 seconds..."); 
      ros::Duration(3.0).sleep(); tried++;
    }
  }
  return 0;
}
