#include "rrt_plugin.h"
#include "ros/ros.h"
#include "cstdint"
#include "csignal"

void shutdown(int sig)
{
    exit(0);
}

void waitRobot(OpenRAVE::RobotBasePtr &robot)
{
    while (robot->GetController()->IsDone() == false)
        usleep(10000);
//        ros::Duration(0.1).sleep();
}

void tuckArms(OpenRAVE::EnvironmentBasePtr &env, OpenRAVE::RobotBasePtr &robot)
{
    std::vector<std::string>        joint_names;
    std::vector<int>                joint_indices;
    std::vector<OpenRAVE::dReal>    dof_values;

    OpenRAVE::ControllerBasePtr     controller;

    joint_names.push_back("l_shoulder_lift_joint");
    joint_names.push_back("l_elbow_flex_joint");
    joint_names.push_back("l_wrist_flex_joint");
    joint_names.push_back("r_shoulder_lift_joint");
    joint_names.push_back("r_elbow_flex_joint");
    joint_names.push_back("r_wrist_flex_joint");

    for (int i = 0; i < joint_names.size(); i++)
        joint_indices.push_back(robot->GetJointIndex(joint_names[i]));

    dof_values.push_back( 1.29023451);
    dof_values.push_back(-2.32099996);
    dof_values.push_back(-0.69800004);
    dof_values.push_back( 1.27843491);
    dof_values.push_back(-2.32100002);
    dof_values.push_back(-0.69799996);

    robot->SetActiveDOFs(joint_indices);
    robot->SetActiveDOFValues(dof_values);

    dof_values.clear();

    controller = robot->GetController();
    robot->GetDOFValues(dof_values);
    controller->SetDesired(dof_values);

    waitRobot(robot);
}

void getInitPosition(std::vector<OpenRAVE::dReal> &dof_values, OpenRAVE::RobotBasePtr &robot, std::vector<std::string> &joint_names)
{
    std::vector<int>                joint_indices;
    std::vector<OpenRAVE::dReal>    all_dof_values;

    joint_names.push_back("l_shoulder_pan_joint");
    joint_names.push_back("l_shoulder_lift_joint");
    joint_names.push_back("l_elbow_flex_joint");
    joint_names.push_back("l_upper_arm_roll_joint");
    joint_names.push_back("l_forearm_roll_joint");
    joint_names.push_back("l_wrist_flex_joint");
    joint_names.push_back("l_wrist_roll_joint");

    for (int i = 0; i < joint_names.size(); i++)
        joint_indices.push_back(robot->GetJointIndex(joint_names[i]));

    dof_values.push_back(-0.15);
    dof_values.push_back(0.075);
    dof_values.push_back(-1.008);
    dof_values.push_back(0);
    dof_values.push_back(0);
    dof_values.push_back(0);
    dof_values.push_back(0);

    robot->SetActiveDOFs(joint_indices);
    robot->SetActiveDOFValues(dof_values);

    dof_values.clear();

    robot->GetDOFValues(all_dof_values);
    for (int i = 0; i < joint_indices.size(); i++)
        dof_values.push_back(all_dof_values.at(joint_indices.at(i)));
}

void setViewer(OpenRAVE::EnvironmentBasePtr p_env, const std::string& viewername)
{
    OpenRAVE::ViewerBasePtr viewer = OpenRAVE::RaveCreateViewer(p_env, viewername);
    BOOST_ASSERT(!!viewer);
    p_env->Add(viewer);
    bool showgui = true;
    viewer->main(showgui);
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "rrt_plugin");
    ros::NodeHandle node_handle;

    std::signal(SIGINT, shutdown);

    OpenRAVE::EnvironmentBasePtr            p_env;
    std::vector<OpenRAVE::RobotBasePtr>     robots;
    OpenRAVE::RobotBasePtr                  robot;
    std::string                             viewername = "qtcoin";
    std::vector<OpenRAVE::KinBodyPtr>       bodies;

    p_env = OpenRAVE::RaveCreateEnvironment();
    p_env->Reset();

    boost::thread   thviewer(boost::bind(setViewer, p_env, viewername));

//    p_env->Load("/usr/share/openrave-0.9/data/pr2test2.env.xml");
    p_env->Load("/home/nbanerjee/Desktop/motion_planning_class/hw3/hw3.env.xml");
    ros::Duration(0.1).sleep();

    p_env->GetRobots(robots);
    robot = robots[0];

    tuckArms(p_env, robot);

//    robot->SetActiveDOFs(std::vector<int>(), OpenRAVE::DOF_X | OpenRAVE::DOF_Y | OpenRAVE::DOF_RotationAxis, OpenRAVE::Vector(0, 0, 1));

    nbRRT::RRTConnect           rrt_connect;
    nbRRT::Configuration        q_init, q_goal;
    std::vector<nbRRT::QValue>  step_size;
    std::vector<nbRRT::QValue>  upper_limits, lower_limits;
    std::vector<std::string>    joint_names;

//    For the hw3 problem
    std::vector<OpenRAVE::dReal>    dof_values;
    static std::vector<OpenRAVE::KinBody::JointPtr> robot_joints;


    getInitPosition(dof_values, robot, joint_names);

    robot->GetActiveDOFLimits(lower_limits, upper_limits);

    robot->GetActiveDOFResolutions(step_size);


    std::vector<int>                joint_indices;

    for (int i = 0; i < joint_names.size(); i++)
        joint_indices.push_back(robot->GetJointIndex(joint_names[i]));

    for (int i = 0; i < lower_limits.size(); i++)
    {
        ROS_INFO_STREAM("Joint name : " << joint_names.at(i));
        ROS_INFO_STREAM("Lower      : " << lower_limits.at(i));
        ROS_INFO_STREAM("Upper      : " << upper_limits.at(i));
        ROS_INFO_STREAM("Resolution : " << step_size.at(i));
    }

    for (int i = 0; i < dof_values.size(); i++)
        q_init.q.push_back(dof_values.at(i));

    q_goal.q.push_back(0.449);
    q_goal.q.push_back(-0.201);
    q_goal.q.push_back(0);
    q_goal.q.push_back(0);
    q_goal.q.push_back(0);
    q_goal.q.push_back(0);
    q_goal.q.push_back(0);

    rrt_connect.setInit(q_init);
    rrt_connect.setGoal(q_goal);
    rrt_connect.setGoalBias(5);
    rrt_connect.setStepSize(step_size);
    rrt_connect.setCollisionCheckFlag(true);
    rrt_connect.setDOFLimits(upper_limits, lower_limits);
    rrt_connect.setEnvironment(p_env);
    rrt_connect.setTimeLimit(180);
    rrt_connect.setJointIndices(joint_indices);
    rrt_connect.setRobot(robot);

    std::vector<nbRRT::Configuration>    solution;

    if (rrt_connect.findPath(solution) == true)
        std::cout << "Done finding path using RRT Connect." << std::endl;
    else
        std::cout << "Could not find a path." << std::endl;       

    while (ros::ok())
        ros::spinOnce();

    thviewer.join();

    p_env->Destroy();

    return 0;
}


//    For the hw2 problem
////    q_init.q.push_back(-3.0);
//    q_init.q.push_back(1.8);
//    q_init.q.push_back(1.8);
//    q_init.q.push_back(0.0);

////    q_goal.q.push_back(2.5);
////    q_goal.q.push_back(0.0);
//    q_goal.q.push_back(-0.8);
//    q_goal.q.push_back(0.1);
//    q_goal.q.push_back(M_PI_2);

//    step_size.push_back(0.05);
//    step_size.push_back(0.05);
//    step_size.push_back(M_PI / 16);

//    upper_limits.push_back(4.0);
//    upper_limits.push_back(4.0);
//    upper_limits.push_back(M_PI);

//    lower_limits.push_back(-4.0);
//    lower_limits.push_back(-4.0);
//    lower_limits.push_back(-M_PI);


//        if (upper_limits.at(i) > 1000)
//            upper_limits.at(i) = 3.0;
//        if (lower_limits.at(i) < -1000)
//            lower_limits.at(i) = 0.0;
