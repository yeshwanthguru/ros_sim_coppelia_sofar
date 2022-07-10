#include "ros/ros.h"
#include "sofar_assignment/Command.h"
#include "sofar_assignment/RandomPosition.h"
#include "sofar_assignment/RcAction.h"
#include "actionlib/client/simple_action_client.h"
#include "actionlib/client/terminal_state.h"

bool start = false;


bool user_interface(sofar_assignment::Command::Request &req, sofar_assignment::Command::Response &res){
    if (req.command == "start"){
    	start = true;
    }
    else {
    	start = false;
    }
    return true;
}

int main(int argc, char **argv)
{
   ros::init(argc, argv, "state_machine");
   ros::NodeHandle n;
   ros::ServiceServer service= n.advertiseService("/user_interface", user_interface);
   ros::ServiceClient client_rp = n.serviceClient<sofar_assignment::RandomPosition>("/position_server");
   actionlib::SimpleActionClient <sofar_assignment::RcAction> pos("/go_to_point", true);
   
   sofar_assignment::RcGoal gp;
   sofar_assignment::RandomPosition rp;
   rp.request.x_max = 5.0;
   rp.request.x_min = -5.0;
   rp.request.y_max = 5.0;
   rp.request.y_min = -5.0;

   while(ros::ok()){
   	ros::spinOnce();
   	if (start){
   		client_rp.call(rp);
        
        gp.x=rp.response.x; 
        gp.y=rp.response.y;  
        gp.theta=rp.response.theta;

        std::cout << "\nGoing to the position: x= " << gp.x << "\nGoing to the position: y= " <<gp.y << " \nGoing to the position:theta = " <<gp.theta << std::endl;
        pos.sendGoal(gp);
        
        while (true)
        {
           ros::spinOnce();
           if(start==false)
           {
                pos.cancelGoal();
                    std::cout << "\n currently goal cancelled" << std::endl;
                    break;
                }
                else
                {
                    if (pos.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
                    {
                        std::cout << "\ncurrently Goal Reached!" << std::endl;
                        break;
                    }
                }
            }
        }
    }
    return 0;
}
