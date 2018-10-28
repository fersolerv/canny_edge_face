#include <iostream>
#include <cv_bridge/cv_bridge.h>
#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>



using namespace std;
using namespace cv;

int low_threshold = 50;
int high_threshold = 100;
int kernelSize = 3;

class ImageCanny {

    ros::NodeHandle imageConverter;
    image_transport::ImageTransport it;
    image_transport::Subscriber sub_img;
    image_transport::Publisher pub_img;

    void publishImage(image_transport::Publisher pub, Mat img) {
        cv_bridge::CvImage imgBridge = cv_bridge::CvImage(std_msgs::Header(), sensor_msgs::image_encodings::RGB8, img);
        sensor_msgs::Image msg;
        imgBridge.toImageMsg(msg);
        pub.publish(msg);
    }

public:

    ImageCanny() : it(imageConverter) {
        sub_img = it.subscribe("/cv_camera/image_raw", 1, &ImageCanny::process, this);
        pub_img = it.advertise("/camera/canny_edge",1);
    }

    void process(const sensor_msgs::ImageConstPtr& srcImg) {
        cv_bridge::CvImagePtr img_ptr;
        try {
            img_ptr = cv_bridge::toCvCopy(srcImg, sensor_msgs::image_encodings::BGR8);
        }
        catch (cv_bridge::Exception& e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }

        Mat img;
        cvtColor(img_ptr->image, img, CV_BGR2GRAY);
        blur(img, img, Size(3, 3));
        Canny(img, img, low_threshold, high_threshold, kernelSize);
        cvtColor(img, img, CV_GRAY2BGR);

        publishImage(pub_img, img);
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "canny_edge_my_face");
    ImageCanny fr;
    while(ros::ok()){
    ros::spin();

}
    return 0;
}
