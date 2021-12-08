#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "zbar.h"

#include "std_msgs/String.h"
using namespace cv;
using namespace zbar;

class ROS_IMG_READER {

  public:
    ROS_IMG_READER() {
      _ros_img_sub = _nh.subscribe("/my_robot/camera1/image_raw", 1, &ROS_IMG_READER::imageCb, this);
      _qr_data_pub = _nh.advertise<std_msgs::String>("/opencv/qr_data", 1);
    }

    void imageCb(const sensor_msgs::ImageConstPtr& msg) {

      cv_bridge::CvImagePtr cv_ptr;
      try {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
      }
      catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
      }
      QR_detect(cv_ptr);
      
    }

    void QR_detect(cv_bridge::CvImagePtr cv_ptr){
      ImageScanner scan;
      scan.set_config(ZBAR_QRCODE,ZBAR_CFG_ENABLE,1); 
      cv::Mat frame=cv_ptr->image;
      cv::Mat frame_gray;
      cv::Mat frame_bw;
      cvtColor(frame,frame_gray,CV_BGR2GRAY);
      cv::threshold(frame_gray,frame_bw, 50,255,cv::THRESH_BINARY);
      u_char*frame_raw = (u_char*)(frame_bw.data);
      // imshow("Display window", frame_bw); // decommentare per vedere immagine filtrata in real time 
      // cv::waitKey(10);
      Image  img(frame.cols,frame.rows,"Y800",frame_raw,frame.rows*frame.cols);
      scan.scan(img);

      for(Image::SymbolIterator x= img.symbol_begin(); x!=img.symbol_end();++x){
        Symbol symb=*x;
        const std::string symb_temp= symb.get_data(); 
        _qr_data_msg.data=symb_temp;
        _qr_data_pub.publish(_qr_data_msg);
      }
  }

  private:
    ros::NodeHandle _nh;
    ros::Subscriber _ros_img_sub; 
    ros::Publisher  _qr_data_pub;
    std_msgs::String _qr_data_msg;

};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "ROS_IMG_READER");
  ROS_IMG_READER ic;
  ros::spin();
  return 0;
}

