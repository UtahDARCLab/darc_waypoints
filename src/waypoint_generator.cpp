// Includes
#include <ros/ros.h>
#include <geometry_msgs/Vector3.h>
#include <vector>
#include <std_msgs/Float32.h>
#include <std_msgs/Int16.h>
#include <sensor_msgs/Joy.h>
#include <string>

geometry_msgs::Vector3 curr_pos, des_pos_out;

double timestep;
int timedLoop;
std::string robotName;

int joy_a, joy_b, joy_x, joy_y;

int startWaypoints;
int landing;
int startControl;
int shape; 

// Read mocap position
void pos_callback(const geometry_msgs::Vector3& pos_msg_in)
{
	curr_pos.x = pos_msg_in.x;
	curr_pos.y = pos_msg_in.y;
	curr_pos.z = pos_msg_in.z;
}

// Read buttons on joystick
void joy_callback(const sensor_msgs::Joy& joy_in)
{
    joy_a = joy_in.buttons[0]; // A gets flying and starts position hold
    joy_b = joy_in.buttons[1]; // B lands and turns off
    joy_x = joy_in.buttons[2]; // X starts waypoints
    joy_y = joy_in.buttons[3]; // Y stops waypoints and holds
    
    if( (joy_a && !startControl) || (joy_b && startControl) )
    {
        startControl = !startControl;
        startWaypoints = 0;
    }
    else if( (joy_x && !startWaypoints) || (joy_y && startWaypoints) )
    {
        startWaypoints = !startWaypoints;
    }
}

double normSquared(geometry_msgs::Vector3 A, geometry_msgs::Vector3 B);
void fillPositionList(std::vector<geometry_msgs::Vector3>&);

int main(int argc, char** argv) 
{
    ros::init(argc,argv,"waypoint_generator");
    ros::NodeHandle node;
    ros::Rate loop_rate(50);
    
    ros::Subscriber pos_sub, joy_sub;
    pos_sub = node.subscribe("current_position",1,pos_callback);
    joy_sub = node.subscribe("joy",1,joy_callback);
    
    ros::Publisher des_pos_pub;
    des_pos_pub = node.advertise<geometry_msgs::Vector3>("desired_position",1);
    
    if ( node.getParam("timed_loop",timedLoop) ) {;}
    else
    {
        ROS_ERROR("Are waypoints timed or position based?");
        return 0;
    }
    if ( timedLoop && node.getParam("waypoint_timestep",timestep) ) {;}
    else
    {
        ROS_ERROR("What is the timing between waypoints?");
        return 0;
    }
    if ( node.getParam("robot_name",robotName) ) {;}
    else
    {
        ROS_ERROR("Which robot number is this?");
        return 0;
    }
    if ( node.getParam("trajectory_shape",shape) ) {;}
    else
    {
        ROS_ERROR("What shape to follow?");
        return 0;
    }
    
    std::vector<geometry_msgs::Vector3> desired_positions;
    
    fillPositionList(desired_positions);
    double bound = 0.05; //0.025;
    des_pos_out = desired_positions[0];
    int max = desired_positions.size()-1;
    int arg = 0;
    int count = 0;
    int countBound = 10;
    
    startWaypoints = 0;
    
    while(ros::ok())
    {
        ros::spinOnce();
        if (startControl && !startWaypoints)
        {
            des_pos_pub.publish(desired_positions[arg]);       
        }
        else if( startControl && startWaypoints )
        {
            if(!timedLoop) // position based loop
            {
                if(normSquared(des_pos_out,curr_pos)<bound*bound)
                {
                    count++;
                }
                if(count > countBound)
                {
                    count = 0;
                    if(arg < max)
                        arg++;
                    else
                        arg = 0;
                    des_pos_out = desired_positions [arg];
                }
            }
            else // time based loop
            {
                count++;
                if( (double)count/100.0 > timestep )
                {
                    count = 0;
                    if(arg < max)
                        arg++;
                    else
                        arg = 0;
                }
            }
            des_pos_pub.publish(desired_positions[arg]);
            ros::spinOnce();
            loop_rate.sleep();
        }
        else
        {
            loop_rate.sleep();
        }
    }
}    
   
void fillPositionList(std::vector<geometry_msgs::Vector3>& posList)
{
    geometry_msgs::Vector3 left, right, front, back, top, bottom, middle;
    geometry_msgs::Vector3 top_left, top_right, bot_left, bot_right;
    double del = 0.4;
    double xCen,yCen,zCen;
    //xCen = 1.21; yCen = -1.3; zCen = 1.2;
    //xCen = 0.0; yCen = 0.0; zCen = 0.0; // Andy crazy flie stuff
    //xCen = 0.0; yCen = 0.0; zCen = 0.3; // Andy crazy flie stuff
    xCen = 0.5; yCen = -1.1; zCen = 0.6; 
    middle.x = xCen;       middle.y = yCen;       middle.z = zCen;
    left.x   = xCen;       left.y   = yCen+del;   left.z   = zCen;
    right.x  = xCen;       right.y  = yCen - del; right.z  = zCen;
    front.x  = xCen + del; front.y  = yCen;       front.z  = zCen;
    back.x   = xCen - del; back.y   = yCen;       back.z   = zCen;
    top.x    = xCen;       top.y    = yCen;       top.z    = zCen + del;
    bottom.x = xCen;       bottom.y = yCen;       bottom.z = zCen - del;
    
    // Square terms
    top_left.x  = xCen + del;     top_left.y = yCen + 1.0*del;     top_left.z = zCen;
    top_right.x = xCen + del;    top_right.y = yCen - 1.0*del;    top_right.z = zCen;
    bot_left.x  = xCen - del;     bot_left.y = yCen + 1.0*del;     bot_left.z = zCen;
    bot_right.x = xCen - del;    bot_right.y = yCen - 1.0*del;    bot_right.z = zCen;
    
    // Tirangle terms
    geometry_msgs::Vector3 tritop, trileft, triright;
    tritop.x = xCen + del; tritop.y = yCen; tritop.z = zCen;
    trileft.x = xCen - 0.5*del; trileft.y = yCen + 0.866*del; trileft.z = zCen;
    triright.x = xCen - 0.5*del; triright.y = yCen - 0.866*del; triright.z = zCen;
        
    
    // ground effects testing terms
    geometry_msgs::Vector3 ground;
    float height = 0.75;//1.25;
    int numSteps = 5;
    float yList[] = {-1.5, -1.0, -0.75, -0.65, -0.60};
    //ground.x = 1.0; ground.y = -1.25; ground.z = height;
    
    if ( robotName == "quad1" )
    {
        if ( shape == 1 ) // square
        {
            posList.push_back(bot_left);
            posList.push_back(top_left);
            posList.push_back(top_right);
            posList.push_back(bot_right);
        }
        else if ( shape == 2 ) //triangle
        {
            posList.push_back(tritop);
            posList.push_back(trileft);
            posList.push_back(triright);
        }
        
        else if (shape == 3) //  proximity effects testing
        {
            ground.x = 1.0; ground.y = -0.5; ground.z = height;
            //for(int n = 0; n < numSteps; n++)
            {               
                //ground.y = yList[n];    
                posList.push_back(ground);
                //ground.z -= height/double(numSteps + 1);
            }
        
        }
    }
    else if ( robotName == "quad2" )
    {
        if( shape == 1 ) // square
        {
            posList.push_back(top_left);
            posList.push_back(top_right);
            posList.push_back(bot_right);
            posList.push_back(bot_left);
        }
        else if ( shape == 2 ) //triangle
        {
            posList.push_back(trileft);
            posList.push_back(triright);
            posList.push_back(tritop);
        }
        else if (shape == 3) // proximity effects testing
        {
            ground.x = 1.0; ground.y = -0.5; ground.z = height;
            for(int n = 0; n < numSteps; n++)
            {
                ground.y = yList[n];       
                posList.push_back(ground);
                //ground.z -= height/double(numSteps + 1);
            }
        
        }
    }
    else if ( robotName == "quad3" )
    {
        if( shape == 1) // square
        {
            posList.push_back(top_right);
            posList.push_back(bot_right);
            posList.push_back(bot_left);
            posList.push_back(top_left);
        }
        else if ( shape == 2 ) // triangle
        {
            posList.push_back(triright);
            posList.push_back(tritop);
            posList.push_back(trileft);
        }
        else if (shape == 3) //  proximity effects testing
        {
            //ground.x = 0.0; ground.y = 0.0; ground.z = height;
            for(int n = 0; n < numSteps; n++)
            {       
                posList.push_back(ground);
                //ground.z -= height/double(numSteps + 1);
            }
        
        }
    }
    
    /*//posList.push_back(bot_left);
    top_left.x = xCen + del;     top_left.y = yCen + del;      top_left.z = zCen + 0.5*del;
    top_right.x = xCen + del;    top_right.y = yCen - del;    top_right.z = zCen + 0.5*del;
    bot_left.x = xCen - del;     bot_left.y = yCen + del;      bot_left.z = zCen + 0.5*del;
    bot_right.x = xCen - del;    bot_right.y = yCen - del;    bot_right.z = zCen + 0.5*del;
    
    posList.push_back(bot_left);
    posList.push_back(top_left);
    posList.push_back(top_right);
    posList.push_back(bot_right);*/
}

double normSquared(geometry_msgs::Vector3 A, geometry_msgs::Vector3 B)
{
    return (B.x-A.x)*(B.x-A.x)+(B.y-A.y)*(B.y-A.y)+(B.z-A.z)*(B.z-A.z);
}
