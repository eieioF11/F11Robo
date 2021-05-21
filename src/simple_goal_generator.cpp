#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

void EulerAnglesToQuaternion(double roll, double pitch, double yaw,
                            double& q0, double& q1, double& q2, double& q3)
{
    double cosRoll = cos(roll / 2.0);
    double sinRoll = sin(roll / 2.0);
    double cosPitch = cos(pitch / 2.0);
    double sinPitch = sin(pitch / 2.0);
    double cosYaw = cos(yaw / 2.0);
    double sinYaw = sin(yaw / 2.0);

    q0 = cosRoll * cosPitch * cosYaw + sinRoll * sinPitch * sinYaw;
    q1 = sinRoll * cosPitch * cosYaw - cosRoll * sinPitch * sinYaw;
    q2 = cosRoll * sinPitch * cosYaw + sinRoll * cosPitch * sinYaw;
    q3 = cosRoll * cosPitch * sinYaw - sinRoll * sinPitch * cosYaw;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "simple_goal_generator");

    //tell the action client that we want to spin a thread by default
    MoveBaseClient ac("move_base", true);

    //wait for the action server to come up
    while (!ac.waitForServer(ros::Duration(5.0)))
    {
        ROS_INFO("Waiting for the move_base action server to come up");
    }

    move_base_msgs::MoveBaseGoal goal;

    double q0,q1,q2,q3;
    double x[]={ 0.0, 0.5, 0.5, 0.0, 0.0};
    double y[]={ 0.0, 0.0,-0.4,-0.4, 0.0};
    double c[]={ 0.0, -90, 180,  90, 0.0};
    int i=0;
    geometry_msgs::Quaternion odom_quat;
    while(true)
    {
        EulerAnglesToQuaternion(0,0,c[i] * M_PI / 180.0,q0,q1,q2,q3);
        odom_quat.w=q0;
        odom_quat.x=q1;
        odom_quat.y=q2;
        odom_quat.z=q3;
        //goal.target_pose.header.frame_id = "base_link";
        goal.target_pose.header.frame_id = "map";
        goal.target_pose.header.stamp = ros::Time::now();

        goal.target_pose.pose.position.x = x[i];
        goal.target_pose.pose.position.y = y[i];
        goal.target_pose.pose.orientation = odom_quat;

        ROS_INFO("Sending goal");
        ROS_INFO("%d(%f,%f)",i,goal.target_pose.pose.position.x,goal.target_pose.pose.position.y);
        ac.sendGoal(goal);

        ac.waitForResult();

        if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        {
            ROS_INFO("Hooray, the base moved 1 meter forward");
            i++;
            if(i>5)
                break;
        }
        else
        {
            ROS_INFO("The base failed to move forward 1 meter for some reason");
            break;
        }
    }
    ROS_INFO("End!!");
    return 0;
}
