#include <stdio.h>
#include <ros/console.h>
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/TwistStamped.h>
#include <control_msgs/GripperCommand.h>

class JoyToTwist
{
public:
    JoyToTwist()
    {
        ros::NodeHandle nh;
        ros::NodeHandle pnh("~");

        pnh.param("linear_scale", linear_scale_, 0.1);   // m/s
        pnh.param("vertical_scale", vertical_scale_, 0.05); // m/s
        pnh.param("angular_scale", angular_scale_, 0.2); // rad/s

        joy_sub_ = nh.subscribe("joy", 10, &JoyToTwist::joyCallback, this);
        twist_pub_ = nh.advertise<geometry_msgs::TwistStamped>("/robot/limb/right/command_twist_stamped", 1);
        grip_pub_ = nh.advertise<control_msgs::GripperCommand>("/robot/gripper/command", 1);

    }
    
private:
    void joyCallback(const sensor_msgs::Joy::ConstPtr& msg)
    {
        // assume the following axes/buttons as present with the CSL gamepad
        // when in Xbox controller mode (home button pressed for 5 seconds)
        
        // axis[0] - left stick horizontal - range left -1 to right +1
        // axis[1] - left stick vertical - range bottom -1 to top +1
        // axis[2] - left trigger - range pressed -1 to unpressed +1
        // axis[3] - right stick horizontal - range left -1 to right +1
        // axis[4] - right stick vertical - range bottom -1 to top +1
        // axis[5] - right trigger - range pressed -1 to unpressed +1
        // axis[6] - cross buttons horizontal- range press left -1 to pressing right +1
        // axis[7] - cross buttons vertical- range press bottom -1 to pressing top +1

        // The triggers of the controller have a signal that starts at 0, once pressed they go to -1, but once unpressed again, they go to +1.
        // The left trigger doesnt reach -1 completely when pressed, it only reaches to aproximately -0.86 
        
        // The buttons are either 1 or 0, 1 when pressed or 0 when not pressed
        // The buttons of the controllers right side are numbered like so: 1-top, 2-right, 3-bottom, 4-left
        // buttons[0] - button 3
        // buttons[1] - button 2
        // buttons[2] - button 4
        // buttons[3] - button 1
        // buttons[4] - left bumper
        // buttons[5] - right bumper
        // buttons[6] - select button
        // buttons[7] - start button
        // buttons[8] - home button
        // buttons[9] - press the left joystick
        // buttons[10] - press the right joystick
        
        
        geometry_msgs::TwistStamped twist, twistSmooth;
        control_msgs::GripperCommand grip;
        twist.header.stamp = ros::Time::now();
        twist.header.frame_id = "base";

        InitializationPhase(msg->axes[2], msg->axes[5], bL, bR);
        
        // Map joystick -> Twist------------------------------------------------------------------------------------
        
        //Left Stick
        if((msg->axes[1] != 0) && (button_pressed[1] == false) && check){
            start_button_press[1] = ros::Time::now();
            button_pressed[1] = true;
        } 
        else if (msg->axes[1] == 0){
            button_pressed[1] = false;
        }
 
        if(button_pressed[1] == true){
            if((ros::Time::now() - start_button_press[1]).toSec() > 1.0){
                twist.twist.linear.x = -linear_scale_ * msg->axes[1] * boost;   // left stick horizontal boosted
            }
            else {
                twist.twist.linear.x = -linear_scale_ * msg->axes[1];   // left stick horizontal
            }
        }

        if((msg->axes[0] != 0) && (button_pressed[0] == false) && check){
            start_button_press[0] = ros::Time::now();
            button_pressed[0] = true;
        } 
        else if (msg->axes[0] == 0){
            button_pressed[0] = false;
        }

        if(button_pressed[0] == true){
            if((ros::Time::now() - start_button_press[0]).toSec() > 1.0){
                twist.twist.linear.y = -linear_scale_ * msg->axes[0] * boost;   // left stick vertical boosted
            }
            else {
                twist.twist.linear.y = -linear_scale_ * msg->axes[0];   // left stick vertical
            }
        }
        
        // Triggers: axes[2] (left), axes[5] (right)
        float left_trigger  = (msg->axes[2]); 
        float right_trigger = (msg->axes[5]);

        // Artificial limit so that both triggers have the same maximum
        if(left_trigger < -0.8){
            left_trigger = -0.8;
        }

        if(right_trigger < -0.8){
            right_trigger = -0.8;
        }
        //
        if(check){
            twist.twist.linear.z = vertical_scale_ * (left_trigger - right_trigger);
        }
            
        //Right Stick
        if((msg->axes[3] != 0) && (button_pressed[3] == false) && check){
            start_button_press[3] = ros::Time::now();
            button_pressed[3] = true;
        } 
        else if (msg->axes[3] == 0){
            button_pressed[3] = false;
        }

        if(button_pressed[3] == true){
            if((ros::Time::now() - start_button_press[3]).toSec() > 1.0){
                twist.twist.angular.x = angular_scale_ * msg->axes[3] * boost;
            }
            else {
                twist.twist.angular.x = angular_scale_ * msg->axes[3];
            }
        }

        if((msg->axes[4] != 0) && (button_pressed[4] == false) && check){
            start_button_press[4] = ros::Time::now();
            button_pressed[4] = true;
        } 
        else if (msg->axes[4] == 0){
            button_pressed[4] = false;
        }

        if(button_pressed[4] == true){
            if((ros::Time::now() - start_button_press[4]).toSec() > 1.0){
                twist.twist.angular.y = -angular_scale_ * msg->axes[4] * boost;
            }
            else {
                twist.twist.angular.y = -angular_scale_ * msg->axes[4];
            }
        }

        //Turn Buttons
        float left_button  = (msg->buttons[4]); 
        float right_button = (msg->buttons[5]);

        if(((msg->buttons[4] != 0) || (msg->buttons[5] != 0)) && (button_pressed[2] == false) && check){
            start_button_press[2] = ros::Time::now();
            button_pressed[2] = true;
        } 
        else if ((msg->buttons[4] == 0) && (msg->buttons[5] == 0)){
            button_pressed[2] = false;
        }

        if(button_pressed[2] == true){
            if((ros::Time::now() - start_button_press[2]).toSec() > 1.0){
                twist.twist.angular.z = angular_scale_ * (left_button - right_button) * boost;
            }
            else {
                twist.twist.angular.z = angular_scale_ * (left_button - right_button);
            }
        }
        
        // Gripper Button-------------------------------------------------------------------------------------------
        if(msg->buttons[1] != LastValueButton)
        {
            LastValueButton = msg->buttons[1];
            grip.position = (1.0 - msg->buttons[1]);
            grip_pub_.publish(grip);
            
        }
        
        //Smoothing the signals-------------------------------------------------------------------------------------
        if (!has_twist_old_)
        {
            twistSmooth = twist;// first message, no filtering
            has_twist_old_ = true;
        }
        else
        {
            twistSmooth.header = twist.header;
            twistSmooth.twist.linear.x  = smooth(twist.twist.linear.x, twistOld.twist.linear.x);
            twistSmooth.twist.linear.y  = smooth(twist.twist.linear.y, twistOld.twist.linear.y);
            twistSmooth.twist.linear.z  = smooth(twist.twist.linear.z, twistOld.twist.linear.z);
            twistSmooth.twist.angular.x = smooth(twist.twist.angular.x, twistOld.twist.angular.x);
            twistSmooth.twist.angular.y = smooth(twist.twist.angular.y, twistOld.twist.angular.y);
            twistSmooth.twist.angular.z = smooth(twist.twist.angular.z, twistOld.twist.angular.z);
        }

        twistOld = twist;
        twist_pub_.publish(twistSmooth);
    }
    
    double smooth(double new_val, double old_val)    {
        return alfa * new_val + (1.0 - alfa) * old_val;
    }
    
    void InitializationPhase(float valueR, float valueL, bool bL, bool bR){
        
        if (!check)
        {
            ROS_WARN_THROTTLE(10, "Press L2 and R2 (both bottom bumpers) to initialize gamepad.");
            if (valueL != 0){
                bL = 1;
            }
            if (valueR != 0){
                bR = 1;
            }
            if (bL && bR){
                check = 1;
            }
        }
    }

    
    ros::Subscriber joy_sub_;
    ros::Publisher twist_pub_;
    ros::Publisher grip_pub_;

    ros::Time start_button_press[5];
    ros::Time time_O, time_Now;
    ros::Duration diff_T;

    geometry_msgs::TwistStamped twistOld;
    
    double linear_scale_;
    double vertical_scale_;
    double angular_scale_;
    
    float LastValueButton = 1.0;
    float alfa = 0.3;
    float boost = 1.5;

    bool has_twist_old_ = false;
    bool button_pressed[5] = {0, 0, 0, 0, 0};
    bool check = false, bL = 0, bR = 0;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "joy_to_twist");
    JoyToTwist node;
    ros::spin();
    return 0;
}