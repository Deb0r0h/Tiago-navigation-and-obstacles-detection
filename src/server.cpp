#include "ros/ros.h"
#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <geometry_msgs/PoseStamped.h>
#include "geometry_msgs/Twist.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf/transform_listener.h>
#include <tf2_ros/transform_listener.h>
#include "sensor_msgs/LaserScan.h"
#include <sensor_msgs/PointCloud.h>
#include <assignment_1_group_12s/TaskAction.h>
#include <cmath>
#include <vector>


using namespace std;
using namespace ros;
using namespace actionlib;

struct Point {
    float x;
    float y;
};

const float THRESHOLD = 0.5; //distance of objects from walls
const float FILL_TRESH = 0.4;// delta of distances to be considered still the object

float angle_increment, angle_min;

// // Function declarations
// vector<float> getLaserRanges(const sensor_msgs::LaserScan::ConstPtr& scan_msg);
// geometry_msgs::Twist calculatePotentialField(const vector<float>& distances, const geometry_msgs::PointStamped& goal_laser);
// geometry_msgs::PointStamped transformGoal(const tf2_ros::Buffer& tfBuffer, Point goal);

//function to check whether the robot is too close to obstacles
bool dangerZone(const vector<float>& data){
    int dataSize = data.size();
    float theta =  (70*M_PI)/180;
    int safetyIndex = static_cast<int>(abs(theta/angle_increment));
     for(int i = dataSize/2-safetyIndex; i < dataSize/2+safetyIndex; i++){
        if (data[i]< 0.5){
            //ROS_INFO("\n \n INTO THE DANGER ZONE \n \n");
            return true;
        }
    }
    return false;
}

//function to check whether the robot has cleared obstacles on the left
bool notClearLeft(const vector<float>& data){
    int dataSize = data.size();
    float theta =  (90*M_PI)/180;
    int safetyIndex = static_cast<int>(abs(theta/angle_increment));
     for(int i = dataSize/2; i < dataSize/2+safetyIndex; i++){
        if (data[i]< 0.5){
            return true;
        }
    }
    return false;
}

//function to check whether the robot has cleared obstacles on the right
bool notClearRight(const vector<float>& data){
    int dataSize = data.size();
    float theta =  (90*M_PI)/180;
    int safetyIndex = static_cast<int>(abs(theta/angle_increment));
     for(int i = dataSize/2-safetyIndex; i < dataSize/2; i++){
        if (data[i]< 0.5){
            return true;
        }
    }
    return false;
}

//Transform the goal position from the map reference frame into the base_laser_link frame
geometry_msgs::PointStamped transformGoal(const tf2_ros::Buffer& tfBuffer, Point goal) {
    // Create a PointStamped message in the /map frame
    geometry_msgs::PointStamped point_map;
    point_map.header.frame_id = "map"; // Source frame
    point_map.header.stamp = ros::Time(0); // Use latest available transform

    // Set the coordinates in the /map frame
    point_map.point.x = goal.x;
    point_map.point.y = goal.y;
    point_map.point.z = 0.0; // Assuming a 2D point

    try {
        // Transform the point to the /base_laser_link frame
        geometry_msgs::PointStamped point_base_laser_link;
        tfBuffer.transform(point_map, point_base_laser_link, "base_laser_link");


        geometry_msgs::Point point = point_base_laser_link.point;
        float x_coordinate = point.x;
        float y_coordinate = point.y;
        float z_coordinate = point.z;

        //ROS_INFO("Current goal position w.r.t. base_laser_link= x: %f, y: %f", x_coordinate, y_coordinate);

        return point_base_laser_link;
    } catch (tf2::TransformException& ex) {
        ROS_WARN("Transform failed: %s", ex.what());
        // Handle the exception, such as returning an invalid point
        return geometry_msgs::PointStamped();
    }
}

// if one bin of the laser reading has a value close enough to the robot to be considered an obstacle,
// the vector of laser data is modified by changing this bin and some neighbouring ones and 
// setting their distance values equal to the one of the "obstacle"
// so the robot doesn't try to "squeeze" through small openings
vector<float> watchForObstacles(const vector<float>& data){
    vector<float> clearedData = data;
    float y_safe = 0.6;
    float x_safe = 0.8;
    float theta =  atan2(y_safe, x_safe);
    int safetyIndex = static_cast<int>(abs(theta/angle_increment));
    int dataSize = data.size();

    for(int i = dataSize/2-safetyIndex; i < dataSize/2+safetyIndex; i++){
        if (data[i]< 0.6){// 0.7
            float obstacle = data[i];
            for(int j = dataSize/2-safetyIndex; j < dataSize/2+safetyIndex; j++){
                clearedData[j] = obstacle;
            }
            return clearedData;
        }
    }
    return clearedData;

}

//function to return the ranges from the laserScan msg
vector<float> getLaserRanges(const sensor_msgs::LaserScan::ConstPtr& scan_msg){
    vector<float> distanceData;
    for(float range : scan_msg->ranges){
        distanceData.push_back(range);
        //ROS_INFO("Laser reading: %f", range);
    }
    //ROS_INFO("END READING");
    return watchForObstacles(distanceData);
}

//function that calculates the best direction (z angle velocity, x axis velocity) 
//based on the goal position and the cleaned laser reading vector
geometry_msgs::Twist calculatePotentialField(const vector<float>& distances, const geometry_msgs::PointStamped& goal_laser) {
    geometry_msgs::Twist vel_msg;
    geometry_msgs::Point goal_point = goal_laser.point;

    int arraySize = static_cast<int>((2*M_PI)/angle_increment);

    //heading angle toward the goal w.r.t. the base_laser_link [-pi,+pi]
    float headingAngle = atan2(goal_point.y, goal_point.x);

    //find the index for the heading angle in the bin vector
    int headingAngleIndex ;
    if(headingAngle>0){//left of x axis, start after the middle of the array
        headingAngleIndex = arraySize/2 + static_cast<int>(abs(headingAngle/angle_increment)); //clockwise angle
    }
    else{//right of x axis, start before the middle of the array
        headingAngleIndex = arraySize/2 - static_cast<int>(abs(headingAngle/angle_increment)); //counterclockwise angle
    }

    //ROS_INFO("heading angle = %f\n" ,headingAngle);
    //ROS_INFO("heading angle index = %d\n" ,headingAngleIndex);

    //compute the attractive potential vector and initialize the repulsive potential vector
    vector<float> attPotential, repPotential, totPotential;
    attPotential.resize(arraySize);
    repPotential.resize(arraySize);

    float K = 0.01; //value by which we decrease the attractive potential
    float value = 1;

    for(int i=headingAngleIndex; i< arraySize; i++){ //left of x axis, positive angles
        attPotential[i] = value;
        value -= K;
        repPotential[i]=0;
    }
    value = 1;
    for(int i=headingAngleIndex-1; i>=0; i--){ //right of x axis, negative angles
        attPotential[i] = value;
        value -= K;
        repPotential[i]=0;
    }


    //compute the repulsive potential vector

    //how many elements we need to shift the distances array for the 360 degrees with same angle_increment
    int indexShift = static_cast<int>((arraySize-distances.size())/2);
    //ROS_INFO("indexShift = %d\n" ,indexShift);

    //create the total potential vector
    totPotential = attPotential;
    float maxPotential = -10;
    int maxPotIndex;
    float repPotGain = 4.0; // 4

    for(int i = 0; i < distances.size(); i++){
        if (distances[i] < 3.0) {
            repPotential[i+indexShift]= 1 - repPotGain*distances[i];
        }
        totPotential[i+indexShift] = totPotential[i+indexShift] - repPotential[i+indexShift];
    }        

    for(int i = 0; i < totPotential.size(); i++){
        if(totPotential[i] > maxPotential){
            maxPotential = totPotential[i];
            maxPotIndex = i;
        }
    }

    //calculate the angle now to follow for the best potential value
    float maxPotAngle;
    //ROS_INFO("Max Potential Angle INDEX : %d \n", maxPotIndex);

    maxPotAngle = (maxPotIndex - arraySize/2)*angle_increment;

    //ROS_INFO("Max Potential Angle : %f \n", maxPotAngle);
    float angleVelGain = 0.6;

    float linearSpeed = 0.3*(M_PI-maxPotAngle)/M_PI*(distances[333]/2);//regulate velocity based on orientatioh
    //ROS_INFO("MlinearSpeed : %f \n", linearSpeed);
    float angularSpeed = (maxPotAngle*angleVelGain)/M_PI;
    //ROS_INFO("angularSpeed : %f \n", angularSpeed);

    //check whether there is an obstacle
    if(dangerZone(distances)){
        if(notClearLeft(distances)){
            linearSpeed = 0;
            angularSpeed = -1.1;
        }
        else if(notClearRight(distances)){
            linearSpeed = 0;
            angularSpeed = 1.1;
        }
        else{ // go back
            linearSpeed = -1;
            angularSpeed = 0;
        }
    }

    vel_msg.linear.x = linearSpeed;
    vel_msg.angular.z = angularSpeed;    
    
    return vel_msg;
}

//function to smooth the laser scan ranges vector
vector<float> smoothScan(const vector<float>& data){
    vector<float> smoothData = data;
    int neighbours = 8; //number of bins to consider
    for(int i=neighbours;i<data.size()-neighbours;i++){
            float newData=0.0;
            for(int j=0;j<2*neighbours+1;j++){
                newData += data[i-neighbours+j];                
            }
            newData = newData/(2*neighbours);
            if(newData<smoothData[i]){ //only update value to make it "worse"
                smoothData[i] =  newData;
            }
        }
    return smoothData;
}

//function to check whether you reached the desired goal position
bool reachedGoal(const geometry_msgs::PointStamped& goal_laser){
    geometry_msgs::Point goal_point = goal_laser.point;
    if(goal_point.x < 0.001 && goal_point.y < 0.001){
        ROS_INFO("GOAL POSITION REACHED! \n");
        return true;
    }
    return false;
}

//function that takes the vector of detected right-edges "cleanDistances" and fills the indexes corresponding
//to the object (within a distance thershold) with their original distance value.
//this approach lead to a vector with all zeros except for when points correspond to objects,
//essentially a vector with distance data of only objects and no walls
vector<float> fill(const vector<float>& cleanDistances,const vector<float>& distances){
    vector<float> filled = cleanDistances;
    float beginningValue;
    for (int i = 0; i < cleanDistances.size(); ++i) {
        if(cleanDistances[i]>0){//there is an edge
            beginningValue = cleanDistances[i];
            while(abs(distances[i+1]-beginningValue)<FILL_TRESH){//cycle until the next point doesn't have distance close enough to be same object
                filled[i+1] = distances[i+1];//save the original distance in the new vector
                //cout << "Filled " << i << ": " << filled[i] << endl;
                i++;
            }
        }
        //cout << "Derivative " << i << ": " << derivative[i] << endl;
        //cout << "Filled " << i << ": " << filled[i] << endl;     
    }

    return filled;
}

//function that takes the raw distance laser data vector and does sort of a right-edge-detection algorithm
//by taking the difference of the distance of a point with the distance of the previous point
//this results in a vector with only non-zero indexes corresponding to the first (right-edge) point of the object
//it then uses this vector "cleanDistances" in the fill function to obtain the vector with only object distance data
vector<float> computeObjVec(const vector<float>& distances){
    vector<float> derivative = distances;
    vector<float> cleanDistances = distances;
    derivative[0]=0;
    cleanDistances[0]=0;
    int count = 0;
    for (int i = 1; i < distances.size(); ++i) {
        derivative[i] = distances[i-1]-distances[i]; //edge-detection/first order discrete derivative
        cleanDistances[i] = distances[i];
        if(distances[i]<0.25 || i < 20 ||i > 646){//points are too close (robot feet) and too on the side
            derivative[i]=0;
            cleanDistances[i]=0;
        }
        else if(derivative[i]<THRESHOLD){//difference is too little
            derivative[i]=0;
            cleanDistances[i]=0;
            }
        else{//right-edge found
            count++;
            }
        //cout << "Derivative " << i << ": " << derivative[i] << endl;
        //cout << "Clean Distances " << i << ": " << cleanDistances[i] << endl;     
    }
    vector<float> filled =  fill(cleanDistances,distances);

    cout << "Number of non negatives : " << count<< endl;
    return filled;
}

//function that calculates the X,Y coordinates of the center point of a circle obtained interpolating three points
Point calculateCircleCenter(const Point& p1, const Point& p2, const Point& p3) {
    float x1 = p1.x, x2 = p2.x, x3 = p3.x;//X coords
    float y1 = p1.y, y2 = p2.y, y3 = p3.y;//Y coords
    float ma = (y2 - y1) / (x2 - x1); //Slope
    float mb = (y3 - y2) / (x3 - x2); //Slope
    // Calculate center coordinates
    Point center;
    center.y = (ma * mb * (y1 - y3) + mb * (x1 + x2) - ma * (x2 + x3)) / (2 * (mb - ma));
    center.x = -1 * (center.y - (x1 + x2) / 2) / ma + (y1 + y2) / 2;
    return center;
}

//funciton to transform polar coordinates into cartsian
Point transformXY(float dist, int ind){
    float theta = angle_min+ind*angle_increment;//w.r.t. x-axis (forward) (negative=clockwise)
    Point point;
    point.x = dist*sin(theta);
    point.y = dist*cos(theta);
    return point;
}

//function that takes all the deteced objects point-clusters and calculates the center point of the cicle for each of them
vector<Point> computeCenterPoints(const vector<float>& distances) {
    //compute the derivative
    vector<float> objectsDist = computeObjVec(distances);
    int firstInd, halfInd, lastInd;
    vector<Point> centers;
    Point first,half,last;
    int numObj =0;
    int count;
    for(int i =0; i< objectsDist.size(); i++){
        if(objectsDist[i]!=0){//objects begin
            count=1;
            firstInd=i;//index of beginning of objects
            while(objectsDist[i+1]!=0){//empties the vector until the objects is finshed
                count++; //counting the number of points of this "object" to discard ones too small
                i++;
            }
            if(count>9 && count <150){
                lastInd = i;//last index of the object
                halfInd = firstInd + round((lastInd-firstInd)/2);//index of a middle point

                numObj++;//number of points of this object

                //compute the X,Y coords for three points
                first = transformXY(objectsDist[firstInd],firstInd);
                half = transformXY(objectsDist[halfInd],halfInd);
                last = transformXY(objectsDist[lastInd],lastInd);

                //interpolate the circle and obtain the X,Y coordinate of the center
                Point center = calculateCircleCenter(first,half,last);
                //push found centers in a vector
                centers.push_back(center);
                
                //-------------------OUTPUTS IN TERMINAL FOUND CENTER OF OBJECTS-------------------
                ROS_INFO("Center of object n.%d of coordinates [%f, %f]",numObj,center.x,center.y);
                //---------------------------------------------------------------------------------
            }
        }
    }
    return centers;
}


// We use this flag in order to execute the callback only once with
// the laser data the robot obtain once arrived at target position.
// Only once is fine since it's stopped in the same point ==> same values of the data.
bool laserCallbackExecuted = false; 
vector<Point> laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg){

    vector<Point> centers;

    if(!laserCallbackExecuted){
        vector<float> distances = msg->ranges;
        angle_min = msg->angle_min;
        angle_increment = msg->angle_increment;

        centers = computeCenterPoints(distances);

        // Update the flag to indicate that the callback has been executed
        laserCallbackExecuted = true;
    }
    return centers;
}

class TaskAction{

    // Declaring Action Server variables
    protected:
        NodeHandle nh_;
        SimpleActionServer<assignment_1_group_12s::TaskAction> as_;
        string action_name;
        assignment_1_group_12s::TaskFeedback feedback_;
        assignment_1_group_12s::TaskResult result_;

    public:
        TaskAction(string name): as_(nh_, name, boost::bind(&TaskAction::execute, this, _1), false), action_name(name){
            as_.start();
        }

    ~TaskAction(void){}


    void execute(const assignment_1_group_12s::TaskGoalConstPtr &goal){
        Rate r(1);   //1 sec
        bool success = true;


        float yaw_angle = goal->point[2];
        int ROBOT_MODE  = goal->point[3];
        // ------------
        // Robot movement handler to target pos
        // ------------

        /////// MOTION CONTROL ///////
        if(ROBOT_MODE == 1){
            feedback_.current_state = "Robot mode: only Motion-control";
            as_.publishFeedback(feedback_);
            ROS_INFO("Publishing feedback: %s", feedback_.current_state.c_str());

            NodeHandle n;

            Point target;
            geometry_msgs::Twist vel_msg;

            //--------------Goal position given by user--------------
            target.x = goal->point[0];
            target.y = goal->point[1];
            //-------------------------------------------------------

            //create a publisher for the cmd_vel topic to control the movement of the robot
            Publisher cmdVelPub = n.advertise<geometry_msgs::Twist>("/mobile_base_controller/cmd_vel", 10);

            //create a buffer and a listener
            tf2_ros::Buffer  tfBuffer(ros::Duration(5.0)); 
            tf2_ros::TransformListener tfListener(tfBuffer);

            feedback_.current_state = "The robot is moving";
            as_.publishFeedback(feedback_);
            ROS_INFO("Publishing feedback: %s", feedback_.current_state.c_str());
            

            Rate loop_rate(100);
            while (ok()) {
                //receive the laserScan message from the topic /scan
                sensor_msgs::LaserScan::ConstPtr laserMsg = topic::waitForMessage<sensor_msgs::LaserScan>("/scan");
                geometry_msgs::TransformStamped transformStamped;

                angle_increment = laserMsg->angle_increment;

                if (laserMsg) {
                    try{
                        transformStamped = tfBuffer.lookupTransform("map", "base_laser_link",Time(0));
                    }
                    catch (tf2::TransformException &ex) {
                        ROS_WARN("%s",ex.what());
                        Duration(0.1).sleep();
                        continue;
                    }

                    //smooth out the ranges vector from the laser scan
                    vector<float> laserRanges = smoothScan(getLaserRanges(laserMsg)); 

                    //obtain the relative position of the Goal with respect to the base_laser_link frame
                    geometry_msgs::PointStamped goal_laser_frame = transformGoal(tfBuffer, target);

                    //if it reached the goal position
                    if(reachedGoal(goal_laser_frame)){ 
                        
                        feedback_.current_state = "The robot reached the Target!";
                        as_.publishFeedback(feedback_);
                        ROS_INFO("Publishing feedback: %s", feedback_.current_state.c_str());

                        SimpleActionClient<move_base_msgs::MoveBaseAction> ac("move_base", true);
                        move_base_msgs::MoveBaseGoal orient_towards_obstacles;  // Creating the move_base goal msg

                        //Set the global map reference frame
                        orient_towards_obstacles.target_pose.header.frame_id = "map";
                        orient_towards_obstacles.target_pose.header.stamp    = Time::now();

                        //Set the position according to the user's prompt
                        orient_towards_obstacles.target_pose.pose.position.x = goal->point[0];
                        orient_towards_obstacles.target_pose.pose.position.y = goal->point[1];
                        orient_towards_obstacles.target_pose.pose.position.z = 0;

                        //Set the orientation
                        tf2::Quaternion orientation;
                        orientation.setRPY(0,0,yaw_angle);   // to let the robot face the obstacles
                        orient_towards_obstacles.target_pose.pose.orientation = tf2::toMsg(orientation);

                        ac.waitForServer();
                        ac.sendGoal(orient_towards_obstacles);

                        // Wait for the result 
                        bool finished_before_timeout = ac.waitForResult(Duration(100.0));

                        if (finished_before_timeout || ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
                            
                            feedback_.current_state = "The robot reached the Target!";
                            as_.publishFeedback(feedback_);
                            ROS_INFO("Publishing feedback: %s", feedback_.current_state.c_str());
                        
                        }else{
                            success = false;
                            feedback_.current_state = "Some errors occured";
                            as_.publishFeedback(feedback_);
                            ROS_INFO("Publishing feedback: %s", feedback_.current_state.c_str());
                            return;
                        }
                        break;
                    }

                    //Publish the velocity command
                    cmdVelPub.publish(calculatePotentialField(laserRanges, goal_laser_frame));  
                    spinOnce();  
                }
                loop_rate.sleep();  
            }

        }else if(ROBOT_MODE == 2){
            feedback_.current_state = "Robot mode: Half Motion-Control, half Navigation";
            as_.publishFeedback(feedback_);
            ROS_INFO("Publishing feedback: %s", feedback_.current_state.c_str());

            NodeHandle n;

            Point checkPoint, target;
            geometry_msgs::Twist vel_msg;

            // Just outside the narrow part
            checkPoint.x = 8.0;
            checkPoint.y = 0.1;

            //--------------Goal position given by user--------------
            target.x = checkPoint.x;
            target.y = checkPoint.y;
            //-------------------------------------------------------

            //create a publisher for the cmd_vel topic to control the movement of the robot
            Publisher cmdVelPub = n.advertise<geometry_msgs::Twist>("/mobile_base_controller/cmd_vel", 10);

            //create a buffer and a listener
            tf2_ros::Buffer  tfBuffer(ros::Duration(5.0)); 
            tf2_ros::TransformListener tfListener(tfBuffer);
            

            Rate loop_rate(100);
            while (ok()) {
                //receive the laserScan message from the topic /scan
                sensor_msgs::LaserScan::ConstPtr laserMsg = topic::waitForMessage<sensor_msgs::LaserScan>("/scan");
                geometry_msgs::TransformStamped transformStamped;

                angle_increment = laserMsg->angle_increment;

                if (laserMsg) {
                    try{
                        transformStamped = tfBuffer.lookupTransform("map", "base_laser_link",Time(0));
                    }
                    catch (tf2::TransformException &ex) {
                         ROS_WARN("%s",ex.what());
                        Duration(0.1).sleep();
                        continue;
                    }

                    //smooth out the ranges vector from the laser scan
                    vector<float> laserRanges = smoothScan(getLaserRanges(laserMsg)); 

                    //obtain the relative position of the Goal with respect to the base_laser_link frame
                    geometry_msgs::PointStamped goal_laser_frame = transformGoal(tfBuffer, target);

                    //if it reached the goal position
                    if(reachedGoal(goal_laser_frame)){ 
                        
                        feedback_.current_state = "The robot reached the Check Point, switching mode..!";
                        as_.publishFeedback(feedback_);
                        ROS_INFO("Publishing feedback: %s", feedback_.current_state.c_str());
                        break;
                    }

                    //Publish the velocity command
                    cmdVelPub.publish(calculatePotentialField(laserRanges, goal_laser_frame));  
                    spinOnce();  
                }
                loop_rate.sleep();  
            }

            r.sleep();
            feedback_.current_state = "Connecting to 'move_base' server..";
            as_.publishFeedback(feedback_);
            ROS_INFO("Publishing feedback: %s", feedback_.current_state.c_str());
            SimpleActionClient<move_base_msgs::MoveBaseAction> ac("move_base", true);

            move_base_msgs::MoveBaseGoal MoveGoal;  // Creating the move_base goal msg
            feedback_.current_state = "Setting robot's target";
            as_.publishFeedback(feedback_);
            ROS_INFO("Publishing feedback: %s", feedback_.current_state.c_str());

            //Set the global map reference frame
            MoveGoal.target_pose.header.frame_id = "map";
            MoveGoal.target_pose.header.stamp    = Time::now();

            //Set the position according to the user's prompt
            MoveGoal.target_pose.pose.position.x = goal->point[0];
            MoveGoal.target_pose.pose.position.y = goal->point[1];
            MoveGoal.target_pose.pose.position.z = 0;

            //Set the orientation
            tf2::Quaternion orientation;
            orientation.setRPY(0,0,yaw_angle);   // to let the robot face the obstacles
            MoveGoal.target_pose.pose.orientation = tf2::toMsg(orientation);

            ac.waitForServer();

            feedback_.current_state = "Sending goal";
            as_.publishFeedback(feedback_);
            ROS_INFO("Publishing feedback: %s", feedback_.current_state.c_str());
            ac.sendGoal(MoveGoal);
            feedback_.current_state = "The robot is moving";
            as_.publishFeedback(feedback_);
            ROS_INFO("Publishing feedback: %s", feedback_.current_state.c_str());

            // Wait for the result 
            bool finished_before_timeout = ac.waitForResult(Duration(100.0));

            if (finished_before_timeout || ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
                
                feedback_.current_state = "The robot reached the Target!";
                as_.publishFeedback(feedback_);
                ROS_INFO("Publishing feedback: %s", feedback_.current_state.c_str());
            
            }else{
                success = false;
                feedback_.current_state = "Some errors occured";
                as_.publishFeedback(feedback_);
                ROS_INFO("Publishing feedback: %s", feedback_.current_state.c_str());
                return;
            }


        }else if(ROBOT_MODE == 3){

            feedback_.current_state = "Robot mode: only Navigation";
            as_.publishFeedback(feedback_);
            ROS_INFO("Publishing feedback: %s", feedback_.current_state.c_str());
            r.sleep();

            feedback_.current_state = "Connecting to 'move_base' server..";
            as_.publishFeedback(feedback_);
            ROS_INFO("Publishing feedback: %s", feedback_.current_state.c_str());
            SimpleActionClient<move_base_msgs::MoveBaseAction> ac("move_base", true);

            move_base_msgs::MoveBaseGoal MoveGoal;  // Creating the move_base goal msg
            feedback_.current_state = "Setting robot's target";
            as_.publishFeedback(feedback_);
            ROS_INFO("Publishing feedback: %s", feedback_.current_state.c_str());

            //Set the global map reference frame
            MoveGoal.target_pose.header.frame_id = "map";
            MoveGoal.target_pose.header.stamp    = Time::now();

            //Set the position according to the user's prompt
            MoveGoal.target_pose.pose.position.x = goal->point[0];
            MoveGoal.target_pose.pose.position.y = goal->point[1];
            MoveGoal.target_pose.pose.position.z = 0;

            //Set the orientation
            tf2::Quaternion orientation;
            orientation.setRPY(0,0,yaw_angle);   // to let the robot face the obstacles
            MoveGoal.target_pose.pose.orientation = tf2::toMsg(orientation);

            ac.waitForServer();

            feedback_.current_state = "Sending goal";
            as_.publishFeedback(feedback_);
            ROS_INFO("Publishing feedback: %s", feedback_.current_state.c_str());
            ac.sendGoal(MoveGoal);
            feedback_.current_state = "The robot is moving";
            as_.publishFeedback(feedback_);
            ROS_INFO("Publishing feedback: %s", feedback_.current_state.c_str());

            // Wait for the result 
            bool finished_before_timeout = ac.waitForResult(Duration(100.0));

            if (finished_before_timeout || ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
                
                feedback_.current_state = "The robot reached the Target!";
                as_.publishFeedback(feedback_);
                ROS_INFO("Publishing feedback: %s", feedback_.current_state.c_str());
            
            }else{
                success = false;
                feedback_.current_state = "Some errors occured";
                as_.publishFeedback(feedback_);
                ROS_INFO("Publishing feedback: %s", feedback_.current_state.c_str());
                return;
            }
        }else{
            feedback_.current_state = "This Robot mode doesn't exists!";
            as_.publishFeedback(feedback_);
            ROS_INFO("Publishing feedback: %s", feedback_.current_state.c_str());
            success = false;
        }

        feedback_.current_state = "The robot started the detection of the obstacles";
        as_.publishFeedback(feedback_);
        ROS_INFO("Publishing feedback: %s", feedback_.current_state.c_str());
        r.sleep();  // To let the client print all the feedbacks 

        // ------------
        // Obstacle detection
        // ------------
        feedback_.current_state = "Obtaining laser data..";
        as_.publishFeedback(feedback_);
        ROS_INFO("Publishing feedback: %s", feedback_.current_state.c_str());

        sensor_msgs::LaserScan::ConstPtr laserMsg = topic::waitForMessage<sensor_msgs::LaserScan>("/scan");

        vector<Point> centers;
        if(laserMsg){
            vector<float> distances = laserMsg->ranges;
            angle_min = laserMsg->angle_min;
            angle_increment = laserMsg->angle_increment;
            centers = computeCenterPoints(distances);
        }

        result_.obstalces.clear();
        for(const auto& center : centers){
            result_.obstalces.push_back(center.x);
            result_.obstalces.push_back(center.y);
        }
        

        if(success){
            feedback_.current_state = "The detection is finished!";
            as_.publishFeedback(feedback_);
            ROS_INFO("Publishing feedback: %s", feedback_.current_state.c_str());

            r.sleep();  // to let all the feedbacks to be printed by the client
            as_.setSucceeded(result_);
        }
    }
};


int main(int argc, char** argv){

    init(argc, argv, "server");
    cout << "Starting the server.." << endl;

    TaskAction task("task");

    spin();
    return 0;
}