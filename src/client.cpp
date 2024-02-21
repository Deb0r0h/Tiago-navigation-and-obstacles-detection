#include "ros/ros.h"
#include "std_msgs/String.h"  
#include <actionlib/client/simple_action_client.h>
#include <assignment_1_group_12s/TaskAction.h>
#include <actionlib/client/terminal_state.h>

using namespace ros;
using namespace std;
using namespace actionlib;


void feedbackCallback(const assignment_1_group_12s::TaskActionFeedbackConstPtr& feedback){
    // Process and print feedback
    ROS_INFO("[Feedback]: %s", feedback->feedback.current_state.c_str());
}


int main(int argc, char **argv){
    if (argc != 4){
        ROS_INFO("Error, correct form: rosrun <your_package_name> <your_node_name> <x> <y> <yaw_angle>");
        return 1;
    }

    init(argc, argv, "client");
    cout << "Starting the client.." << endl;

    // Setup a feedback callback
    NodeHandle feedback_nh;
    Subscriber feedback_sub = feedback_nh.subscribe("task/feedback", 10, feedbackCallback);

    // Create an Action client
    SimpleActionClient<assignment_1_group_12s::TaskAction> ac("task", true);
    ROS_INFO("Waiting the 'task Server' to start..");

    ac.waitForServer(); //will wait for infinite time

    // Setting the Goal according to user's Input
    ROS_INFO("'task Server' started, sending goal.");

    assignment_1_group_12s::TaskGoal goal;
    

    goal.point.push_back(atof(argv[1]));
    goal.point.push_back(atof(argv[2]));
    goal.point.push_back(atof(argv[3]));

    // Mode-switch
    int ROBOT_MODE;
    // Prompt the user for input
    cout << "Choose the robot's behaviour mode:\n'1': only Motion-control\n'2': Half Motion-Control, half Navigation\n'3': only Navigation\n";
    // Get input from the user
    cin >> ROBOT_MODE;
    while (cin.fail() || (ROBOT_MODE != 1 && ROBOT_MODE != 2 && ROBOT_MODE != 3)) {
        cin.clear();  // Clear the error flag
        cin.ignore(numeric_limits<streamsize>::max(), '\n');  // Discard invalid input
        cerr << "Error: Invalid input. Please enter an integer." << endl;
        cin >> ROBOT_MODE;  //retry
    }
    goal.point.push_back(ROBOT_MODE);

    ac.sendGoal(goal);

    // Set this while to allow the feedback_nh to subscribe the topic and print the feedback
    Rate r(10);
    while(ok() && !ac.waitForResult(Duration(0.1))){
        spinOnce();
        r.sleep();
    }

    assignment_1_group_12s::TaskResultConstPtr result;

    if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED || ac.waitForResult()){
        ROS_INFO("Assignment 1 task finished: [SUCCEEDED]\nPrinting results..");

        result = ac.getResult();
        for(int i=0; i<result->obstalces.size()-1; i+=2){
            ROS_INFO("[Result] Obstacle [%d]: (%lf, %lf)", (i/2)+1, result->obstalces[i], result->obstalces[i+1]);
        }
    }else{
        ROS_INFO("Action did have some problems.");
        ac.cancelGoal();
    }

    return 0;
}