#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Float64MultiArray.h>

#include <opencv2/core.hpp>
#include <sstream>
#include <vector>

#include "HoF_Raster.hpp"


ros::Publisher hist0_pub;
ros::Publisher hist2_pub;


void pubHist(ros::Publisher& pub, double* hist)
{
    // std::stringstream ss;
    // for (int i = 0; i < 181; i++)
    // {
    //     ss << hist[i] << ",";
    // }
    // ROS_INFO_STREAM(ss.str());

    std_msgs::Float64MultiArray msg;
    
    std::vector<double> h_vec(hist, hist+181);
    msg.data = h_vec;
    
    pub.publish(msg);
}


void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv::Mat image = cv_ptr->image;

    ROS_INFO_STREAM("Computing HoFs on image (" << image.size << ")");

    cv::Mat imageAM, imageBM;
    cv::extractChannel(image, imageAM, 0);
    cv::extractChannel(image, imageBM, 2);

    // ROS_INFO_STREAM("ImageA (" << imageAM.size << "," << imageAM.channels() << ")");
    // ROS_INFO_STREAM("ImageB (" << imageBM.size << "," << imageBM.channels() << ")");

    int M = image.rows;
    int N = image.cols;

    unsigned char *imageA = imageAM.data;
    unsigned char *imageB = imageBM.data;

    double CONSTANT_FORCE = 0;
    double GRAVITATIONAL_FORCE = 2;

    int numberDirections = 180;
    double f0_histogram[181];
    double f2_histogram[181];

    hof::HoF_Raster raster_obj;
    raster_obj.FRHistogram_CrispRaster(f0_histogram, numberDirections, CONSTANT_FORCE, imageA, imageB, M, N);
    raster_obj.FRHistogram_CrispRaster(f2_histogram, numberDirections, GRAVITATIONAL_FORCE, imageA, imageB, M, N);

    pubHist(hist0_pub, f0_histogram);
    pubHist(hist2_pub, f2_histogram);
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "hof_node");
    ros::NodeHandle n;
    image_transport::ImageTransport it(n);

    image_transport::Subscriber image_sub = it.subscribe("/hof_img", 1, imageCallback);

    hist0_pub = n.advertise <std_msgs::Float64MultiArray>("/f0_histogram", 1000);
    hist2_pub = n.advertise <std_msgs::Float64MultiArray>("/f2_histogram", 1000);
    
    ros::Rate loop_rate(10);

    ros::spin();

    return 0;
}