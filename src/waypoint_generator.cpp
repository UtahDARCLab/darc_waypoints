// Includes
#include <ros/ros.h>
#include <geometry_msgs/Vector3.h>
#include <vector>

geometry_msgs::Vector3 curr_pos, des_pos_out;


// Read mocap position
void pos_callback(const geometry_msgs::Vector3& pos_msg_in)
{
	curr_pos.x = pos_msg_in.x;
	curr_pos.y = pos_msg_in.y;
	curr_pos.z = pos_msg_in.z;
}

double normSquared(geometry_msgs::Vector3 A, geometry_msgs::Vector3 B);
void fillPositionList(std::vector<geometry_msgs::Vector3>&);

int main(int argc, char** argv) 
{
    ros::init(argc,argv,"waypoint_generator");
    ros::NodeHandle node;
    ros::Rate loop_rate(50);
    
    ros::Subscriber pos_sub;
    pos_sub = node.subscribe("current_position",1,pos_callback);
    
    ros::Publisher des_pos_pub;
    des_pos_pub = node.advertise<geometry_msgs::Vector3>("desired_position",1);
    
    std::vector<geometry_msgs::Vector3> desired_positions;
    fillPositionList(desired_positions);
    double bound = 0.2;
    des_pos_out = desired_positions[0];
    int max = desired_positions.size()-1;
    int arg = 0;
    while(ros::ok())
    {
        if(normSquared(des_pos_out,curr_pos)<bound*bound)
        {
            if(arg < max)
                arg++;
            else
                arg = 0;
            des_pos_out = desired_positions [arg];
            ROS_INFO("arg: %d",arg);
            //ROS_INFO("Des Pos: %f, %f, %f",des_pos_out.x,des_pos_out.y,des_pos_out.z);
        }

        des_pos_pub.publish(des_pos_out);
        ros::spinOnce();
        loop_rate.sleep();
    }
}    
   
void fillPositionList(std::vector<geometry_msgs::Vector3>& posList)
{
    geometry_msgs::Vector3 left, right, front, back, top, bottom, middle;

    double del = 0.7;
    double xCen,yCen,zCen;
    xCen = 1.21; yCen = -1.3; zCen = 1.2;

    middle.x = xCen;       middle.y = yCen;       middle.z = zCen;
    left.x   = xCen;       left.y   = yCen+del;   left.z   = zCen;
    right.x  = xCen;       right.y  = yCen - del; right.z  = zCen;
    front.x  = xCen + del; front.y  = yCen;       front.z  = zCen;
    back.x   = xCen - del; back.y   = yCen;       back.z   = zCen;
    top.x    = xCen;       top.y    = yCen;       top.z    = zCen + del;
    bottom.x = xCen;       bottom.y = yCen;       bottom.z = zCen - del;
    
    /*posList.push_back(left);
    posList.push_back(front);
    posList.push_back(right);
    posList.push_back(back);
    posList.push_back(top);
    posList.push_back(bottom);*/
    posList.push_back(middle);
    
}

double normSquared(geometry_msgs::Vector3 A, geometry_msgs::Vector3 B)
{
    return (B.x-A.x)*(B.x-A.x)+(B.y-A.y)*(B.y-A.y)+(B.z-A.z)*(B.z-A.z);
}