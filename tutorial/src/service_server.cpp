// ROS
#include <ros/ros.h>
#include <ros/package.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <tutorial/my_service.h>
// OpenCV
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

class MyServer
{
  private:
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    ros::ServiceServer service_;
    cv_bridge::CvImagePtr cv_ptr_;
    cv::CascadeClassifier face_cascade_;
    std::string package_path_, face_cascade_name_;
    // /usb_cam/image_raw callback
    // save the image in cv_ptr_
    void imageCb(const sensor_msgs::ImageConstPtr& msg){
      try{
        cv_ptr_ = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
      }
      catch (cv_bridge::Exception& e){
        ROS_ERROR("cv_bridge excpetion: %s", e.what());
        return;
      }
    }
    // my_service callback
    // Detect how many faces in the image and return the information in string
    // If the number is not zero, save the image with given filename
    bool serviceCb(tutorial::my_service::Request &req, 
                   tutorial::my_service::Response &res){
      cv::Mat frame_with_face = cv_ptr_->image;
      int face_num = face_detect(cv_ptr_->image, frame_with_face);
      std::stringstream ss;
      ss << face_num;
      res.result = ss.str() + " face" + (face_num>1?"s": "") + " in image";
      if (face_num != 0) {cv::imwrite(package_path_ + "/image/" + req.filename + ".jpg", frame_with_face);}
      return true;
    }
    // Detect face using CV, refer to:
    // https://docs.opencv.org/2.4/doc/tutorials/objdetect/cascade_classifier/cascade_classifier.html
    int face_detect(cv::Mat frame, cv::Mat& frame_with_face){
      std::vector<cv::Rect> faces;
      // Convert to gray image and equalize the histogram
      cv::Mat frame_gray;
      cv::cvtColor(frame, frame_gray, CV_BGR2GRAY);
      cv::equalizeHist(frame_gray, frame_gray);
      // Detect faces
      face_cascade_.detectMultiScale(frame_gray, faces, 1.1, 2, 0|CV_HAAR_SCALE_IMAGE, cv::Size(30, 30));
      // Draw circles to bounded the detected faces
      for (size_t i = 0; i < faces.size(); ++i){
        cv::Point center(faces[i].x + faces[i].width * 0.5, faces[i].y + faces[i].height * 0.5);
        cv::circle(frame_with_face, center, cvRound((faces[i].width + faces[i].height) * 0.25), cv::Scalar(255, 0, 0), 4, 8, 0);
      }
      // Return the number of face
      return (int)faces.size();
    }
        
  public:
    MyServer(): it_(nh_){
      // Get package path, to know where to load the config file and save image
      package_path_ = ros::package::getPath("tutorial");
      face_cascade_name_ = package_path_ + "/config" +"/haarcascade_frontalface_alt.xml";
      // Check if successfully load config file
      if(!face_cascade_.load(face_cascade_name_)) {ROS_ERROR("Error loading config file, end the process..."); ros::shutdown();}
      else ROS_INFO("Successfully loading config file, start to detect face...");
      image_sub_ = it_.subscribe("/usb_cam/image_raw", 1, 
                                 &MyServer::imageCb, this);
      service_ = nh_.advertiseService("my_service", 
                                      &MyServer::serviceCb, this);
    }
    ~MyServer() {}
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "service_server_node");
  MyServer ms;
  ros::spin();
  return 0;
}
