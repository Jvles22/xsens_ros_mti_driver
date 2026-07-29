#ifndef IMUPUBLISHER_H
#define IMUPUBLISHER_H

#include "packetcallback.h"
#include <sensor_msgs/Imu.h>

static void variance_from_stddev_param(std::string param, double *variance_out)
{
    std::vector<double> stddev;
    if (ros::param::get(param, stddev))
    {
        if (stddev.size() == 3)
        {
            auto squared = [](double x) { return x * x; };
            std::transform(stddev.begin(), stddev.end(), variance_out, squared);
        }
        else
        {
            ROS_WARN("Wrong size of param: %s, must be of size 3", param.c_str());
        }
    }
    else
    {
        memset(variance_out, 0, 3 * sizeof(double));
    }
}

struct ImuPublisher : public PacketCallback
{
    ros::Publisher pub;
    std::string frame_id = DEFAULT_FRAME_ID;

    double orientation_variance[3];
    double linear_acceleration_variance[3];
    double angular_velocity_variance[3];

    ImuPublisher(ros::NodeHandle &node)
    {
        int pub_queue_size = 5;
        ros::param::get("~publisher_queue_size", pub_queue_size);
        pub = node.advertise<sensor_msgs::Imu>("imu/data", pub_queue_size);
        ros::param::get("~frame_id", frame_id);

        variance_from_stddev_param("~orientation_stddev", orientation_variance);
        variance_from_stddev_param("~angular_velocity_stddev", angular_velocity_variance);
        variance_from_stddev_param("~linear_acceleration_stddev", linear_acceleration_variance);
    }

    void operator()(const XsDataPacket &packet, ros::Time timestamp)
    {
        bool quaternion_available = packet.containsOrientation();
        bool gyro_available       = packet.containsCalibratedGyroscopeData();
        bool accel_available      = packet.containsCalibratedAcceleration();

        geometry_msgs::Quaternion quaternion;
        if (quaternion_available)
        {
            XsQuaternion q = packet.orientationQuaternion();
            // Rotation 180deg autour de X : corrige repere IMU (Z vers le haut)
            // q_corrected = q_rot180x * q_original
            // q_rot180x = (w=0, x=1, y=0, z=0)
            double qw = q.w(), qx = q.x(), qy = q.y(), qz = q.z();
            quaternion.w = -qx;
            quaternion.x =  qw;
            quaternion.y = -qz;
            quaternion.z =  qy;
        }

        geometry_msgs::Vector3 gyro;
        if (gyro_available)
        {
            XsVector g = packet.calibratedGyroscopeData();
            gyro.x =  g[0];
            gyro.y = -g[1];  // correction repere
            gyro.z = -g[2];  // correction repere
        }

        geometry_msgs::Vector3 accel;
        if (accel_available)
        {
            XsVector a = packet.calibratedAcceleration();
            accel.x =  a[0];
            accel.y = -a[1];  // correction repere
            accel.z = -a[2];  // correction repere
        }

        if (quaternion_available || accel_available || gyro_available)
        {
            sensor_msgs::Imu msg;
            msg.header.stamp    = timestamp;
            msg.header.frame_id = frame_id;

            msg.orientation = quaternion;
            if (quaternion_available)
            {
                msg.orientation_covariance[0] = orientation_variance[0];
                msg.orientation_covariance[4] = orientation_variance[1];
                msg.orientation_covariance[8] = orientation_variance[2];
            }
            else
            {
                msg.orientation_covariance[0] = -1;
            }

            msg.angular_velocity = gyro;
            if (gyro_available)
            {
                msg.angular_velocity_covariance[0] = angular_velocity_variance[0];
                msg.angular_velocity_covariance[4] = angular_velocity_variance[1];
                msg.angular_velocity_covariance[8] = angular_velocity_variance[2];
            }
            else
            {
                msg.angular_velocity_covariance[0] = -1;
            }

            msg.linear_acceleration = accel;
            if (accel_available)
            {
                msg.linear_acceleration_covariance[0] = linear_acceleration_variance[0];
                msg.linear_acceleration_covariance[4] = linear_acceleration_variance[1];
                msg.linear_acceleration_covariance[8] = linear_acceleration_variance[2];
            }
            else
            {
                msg.linear_acceleration_covariance[0] = -1;
            }

            pub.publish(msg);
        }
    }
};

#endif
