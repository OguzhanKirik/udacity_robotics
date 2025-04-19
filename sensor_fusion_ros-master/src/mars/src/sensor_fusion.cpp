#include "ros/ros.h"
#include "std_msgs/String.h"

#include "mars/IRStamped.h"
#include "geometry_msgs/TwistWithCovarianceStamped.h"
#include "geometry_msgs/PointStamped.h"
#include <cmath>

#include <message_filters/subscriber.h>
#include <message_filters/cache.h>
#include "geometry_msgs/PoseWithCovarianceStamped.h"

#include <ros/console.h>



 class Sensor{
    private:
      ros::Subscriber sub_radar; // Subscriber to radar data
      ros::Subscriber sub_samples; // Subscriber to beam data
      ros::Publisher pub_position; // Publisher to estimated position
      geometry_msgs::PoseWithCovarianceStamped position_radar;
      geometry_msgs::PointStamped position_estimated;

      double time_new, time_intervall,time_last;
      int counter; 
    public:
        Sensor(ros::NodeHandle& n){
            ROS_INFO("sensor fusion node is ready");
            // Subscribing to ir_samples and radar_samples;
            sub_samples =n.subscribe("ir_samples",1000, &Sensor::sample_position,this); 
            sub_radar =n.subscribe("radar_samples",1000, &Sensor::radar,this);
            pub_position = n.advertise<geometry_msgs::PointStamped>("estimated_position", 10);
            // initialiting the position
            position_radar.pose.pose.position.x = 0;
            counter = 0;
        }
        ~Sensor(){};

        void sample_position(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg){
            position_estimated.header = msg->header;
            //measurement
            // Receving the observation info from Ir sensor
            // Calculating the positional discrepency between measurement and prediciton
            // calculate the kalman gain based on the covarinaces
            ROS_INFO("Measurement received, calculating ...");

            double diff = msg->pose.pose.position.x - position_radar.pose.pose.position.x;
            double Kalman_Gain  = position_radar.pose.covariance[0]/ (msg->pose.covariance[0]+position_radar.pose.covariance[0]);
            // Update
            // Update the positon with kalman gain and difference
            ROS_INFO("Updating the position ...");

            position_estimated.point.x = position_radar.pose.pose.position.x + (Kalman_Gain * diff);
            pub_position.publish(position_estimated);    
        }

     

          void radar(const geometry_msgs::TwistWithCovarianceStamped::ConstPtr& msg_radar){
              // Predict
              // Calculating the ego vehicle pose by multiplying the velocity and time between messages
            if(counter  == 0){
              this->time_last = msg_radar->header.stamp.toSec();
            }else{
              position_radar.header = msg_radar->header;
              position_radar.pose.covariance = msg_radar->twist.covariance;
              this->time_new = msg_radar->header.stamp.toSec();
              this->time_intervall = (this->time_new - this->time_last);
              this->position_radar.pose.pose.position.x += ((msg_radar->twist.twist.linear.x * time_intervall)  );
              this->time_last = this->time_new;  
            }
            counter++;
        }


};



int main(int argc, char **argv){

    ros::init(argc,argv, "sensor_fusion");
    ros::NodeHandle n;
    Sensor sensor(n);    
    ros::spin();
  
    return 0;
}