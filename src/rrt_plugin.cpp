#include "rrt_plugin.h"

namespace nbRRT
{
    // ##############################################################################
    //                         class(Configuration) definitions
    // ##############################################################################

    Configuration::Configuration()
    {
        // ...
    }

    Configuration::~Configuration()
    {
        // ...
    }

    void Configuration::operator =(const Configuration &C)
    {
        q = C.q;        
    }

    bool Configuration::operator ==(const Configuration &C)
    {
        if (q == C.q)
            return true;
        else
            return false;
    }

    // ##############################################################################
    //                           class(RRTNode) definitions
    // ##############################################################################

    RRTNode::RRTNode()
    {
        _parent = -1;
        _index = -1;
    }

    RRTNode::~RRTNode()
    {
        // ...
    }

    void RRTNode::operator =(const RRTNode &N)
    {
        _configuration = N._configuration;
        _parent = N._parent;
        _index = N._index;
    }

    bool RRTNode::operator ==(const RRTNode &N)
    {
        if (_configuration == N._configuration)
            return true;
        else
            return false;
    }

    // ##############################################################################
    //                           class(NodeTree) definitions
    // ##############################################################################

    NodeTree::NodeTree()
    {
        // ...
    }

    NodeTree::~NodeTree()
    {
        // ...
    }

    void NodeTree::init(RRTNode q)
    {
        _nodes.clear();
        _nodes.push_back(q);
        _initialised = true;
    }

    bool NodeTree::addVertex(RRTNode q)
    {
        if (_nodes.size() == 0)
            return false;
        q._index = _nodes.size();
        _nodes.push_back(q);
        return true;
    }

    bool NodeTree::addEdge(RRTNode q1, RRTNode q2)
    {
        bool    found_q1, found_q2;
        int     index1, index2;

        found_q1 = found_q2 = false;
        for (int i = 0; i < _nodes.size(); i++)
        {
            if ((found_q1 == true) && (found_q2 == true))
                break;

            if (_nodes[i] == q1)
            {
                index1 = i;
                found_q1 = true;
            }

            if (_nodes[i] == q2)
            {
                index2 = i;
                found_q2 = true;
            }
        }

        if ((found_q1 == true) && (found_q2 == true))
        {
            _nodes[index2]._parent = index1;
            return true;
        }
        else
            return false;
    }

    void NodeTree::getPath(RRTNode from_node, std::vector<int> &indices)
    {
        RRTNode cur = from_node;

        for (int i = 0; i < _nodes.size(); i++)
            if (_nodes[i] == cur)
            {
                cur = _nodes[i];
                break;
            }

        while ((cur._index != -1))
        {
            indices.push_back(cur._index);
            cur = _nodes[cur._parent];
        }
    }

    // ##############################################################################
    //                         class(RRTConnect) definitions
    // ##############################################################################

    RRTConnect::RRTConnect()
    {
        _result = false;
        _collision_check = false;
        _seconds = 180;

//        RegisterCommand("setInit", boost::bind(&RRTConnect::setInit, this, _1, _2), "returns nothing");
//        RegisterCommand("setGoal", boost::bind(&RRTConnect::setGoal, this, _1, _2), "returns nothing");
//        RegisterCommand("setStepSize", boost::bind(&RRTConnect::setStepSize, this, _1, _2), "returns nothing");
//        RegisterCommand("setGoalBias", boost::bind(&RRTConnect::setGoalBias, this, _1, _2), "returns nothing");
//        RegisterCommand("setDOFLimits", boost::bind(&RRTConnect::setDOFLimits, this, _1, _2), "returns nothing");
//        RegisterCommand("setEnvironment", boost::bind(&RRTConnect::setEnvironment, this, _1, _2), "returns nothing");
//        RegisterCommand("setRobot", boost::bind(&RRTConnect::setRobot, this, _1, _2), "returns nothing");
//        RegisterCommand("setTimeLimit", boost::bind(&RRTConnect::setTimeLimit, this, _1, _2), "returns nothing");
//        RegisterCommand("findPath", boost::bind(&RRTConnect::findPath, this, _1, _2), "returns nothing");
    }

    RRTConnect::~RRTConnect()
    {
        // ...
    }

    void RRTConnect::setInit(Configuration &q_init)
    {
        _init._configuration = q_init;
        _init._parent = -1;
        _dimensions = q_init.q.size();
    }

    void RRTConnect::setGoal(Configuration &q_goal)
    {
        _goal._configuration = q_goal;
        _goal._parent = -1;
    }

    void RRTConnect::setStepSize(std::vector<QValue> &step_size)
    {
        _step_size = step_size;
    }

    void RRTConnect::setGoalBias(int bias)
    {
        _goal_bias = bias;
    }

    void RRTConnect::setCollisionCheckFlag(bool enable)
    {
        _collision_check = enable;
    }

    void RRTConnect::setDOFLimits(std::vector<QValue> &upper_limits, std::vector<QValue> &lower_limits)
    {
        _q_upper_limits = upper_limits;
        _q_lower_limits = lower_limits;
    }

    void RRTConnect::setEnvironment(OpenRAVE::EnvironmentBasePtr &env)
    {
        _env = env;
    }    

    void RRTConnect::setRobot(OpenRAVE::RobotBasePtr &robot)
    {
        _robot = robot;
    }

    void RRTConnect::setTimeLimit(int seconds)
    {
        _seconds = seconds;
    }

    void RRTConnect::setJointIndices(std::vector<int> &joint_indices)
    {
        _joint_indices = joint_indices;
    }

    bool RRTConnect::findPath(std::vector<Configuration> &solution)
    {
        _result = false;
        buildRRT(_init);
        if (_result == true)
        {
            _tree.getPath(_goal, _solution_path);
            shortcutSmoothing();
//            std::cout << "Nodes searched: " << _tree._nodes.size();
            std::cout << "Size of solution path: " << _solution_path.size() << std::endl;            
            std::cout << "Size of shortcut path: " << _smoothed_path.size() << std::endl;
            for (int i = 0; i < _solution_path.size(); i++)
            {
//                solution.push_back(_tree._nodes[_solution_path[_solution_path.size() - i - 1]]._configuration);
                drawNode(_tree._nodes[_solution_path[_solution_path.size() - i - 1]]._configuration, 0);
            }
            for (int i = 0; i < _smoothed_path.size(); i++)
            {
                solution.push_back(_tree._nodes[_smoothed_path[_smoothed_path.size() - i - 1]]._configuration);
                drawNode(_tree._nodes[_smoothed_path[_smoothed_path.size() - i - 1]]._configuration, 1);
            }
        }
        std::cout << "Nodes searched: " << _tree._nodes.size();
        return _result;
    }

    void RRTConnect::buildRRT(RRTNode q_init)
    {
        std::chrono::time_point<std::chrono::system_clock>  start, end;
        std::chrono::duration<double>                       elapsed_seconds;

        RRTNode     q_rand;
        int         K = std::numeric_limits<int>::max();
        ExtendModes mode;

        start = std::chrono::system_clock::now();
        _tree.init(q_init);
        for (int k = 0; k < K; k++)
        {
            end = std::chrono::system_clock::now();
            elapsed_seconds = end - start;
            if (elapsed_seconds.count() > _seconds)
                return;            

            randomConfig(q_rand);
            mode = connect(q_rand);
            if (q_rand == _goal)
                if (mode == ExtendModes::Reached)
                {
                    std::cout << "Final time: " << elapsed_seconds.count() << std::endl;
                    _result = true;
                    return;
                }
        }        
    }

    ExtendModes RRTConnect::extend(RRTNode &q)
    {
        RRTNode q_near, q_new;

        q_near = nearestNeighbour(q);
        if (newConfig(q, q_near, q_new) == true)
        {
            _tree.addVertex(q_new);
            _tree.addEdge(q_near, q_new);
            if (q_new == q)
                return ExtendModes::Reached;
            else
                return ExtendModes::Advanced;
        }

        return ExtendModes::Trapped;
    }

    ExtendModes RRTConnect::connect(RRTNode &q)
    {
        ExtendModes S = ExtendModes::Advanced;

        while (S == ExtendModes::Advanced)
            S = extend(q);

        drawNearestNeighbourToGoal();

        return S;
    }

    void RRTConnect::randomConfig(RRTNode &q_rand)
    {
        static int                                          randomiser_initialised = false;
        static std::vector<OpenRAVE::KinBody::JointPtr>     robot_joints;
        double                                              q_sampled;

        if (randomiser_initialised == false)
        {
            int time = std::time(0);
//            std::srand(time);
//            std::srand(1427150762);
            std::srand(1427142869);
//            std::srand(1427018784);
//            std::srand(1427077258);
            // 1427014768
//            1427018784
            std::cout << "Seed: " << time << std::endl;
            randomiser_initialised = true;
            q_rand = _goal;
            robot_joints = _robot->GetJoints();
            for (int i = 0; i < robot_joints.size(); i++)
            {
                for (int j = 0; j < _joint_indices.size(); j++)
                {
                    if (_joint_indices[j] == robot_joints[i]->GetDOFIndex())
                    {
                        if (((OpenRAVE::KinBody::JointPtr) robot_joints[i])->IsCircular(0) == true)
                        {
                            _q_upper_limits[j] = M_PI;
                            _q_lower_limits[j] = 0;
                            std::cout << "Limit change: " << robot_joints[i]->GetName() << std::endl;
                        }
                    }
                }
            }

            std::cout << _joint_indices.size() << std::endl;

            for (int i = 0; i < _joint_indices.size(); i++)
                std::cout << "Limits: " << _q_upper_limits[i] << " - " << _q_lower_limits[i] << std::endl;
            return;
        }        

        q_sampled = std::rand() % 100;
        if (((int) q_sampled) < _goal_bias)
        {
            q_rand = _goal;
            return;
        }

        q_rand._parent = -1;
        q_rand._configuration.q.clear();

        for (int i = 0; i < _dimensions; i++)
        {
            q_sampled = std::rand() % ((int) ((_q_upper_limits.at(i) - _q_lower_limits.at(i)) * 100));
            q_sampled = q_sampled / 100;
            q_sampled = _q_lower_limits.at(i) + q_sampled;
            q_rand._configuration.q.push_back(q_sampled);
        }
    }

    RRTNode RRTConnect::nearestNeighbour(RRTNode &q)
    {
        double l2_dist_min = std::numeric_limits<double>::max();
        double l2_dist_cur = 0;
        RRTNode q_near;

        for (int i = 0; i < _tree._nodes.size(); i++)
        {
            for (int j = 0; j < _dimensions; j++)
                l2_dist_cur = l2_dist_cur + (_dimensions - j)*std::pow(_tree._nodes.at(i)._configuration.q.at(j) - q._configuration.q.at(j), 2);

            if (l2_dist_cur < l2_dist_min)
            {
                l2_dist_min = l2_dist_cur;
                q_near = _tree._nodes.at(i);
            }

            l2_dist_cur = 0;
        }

        return q_near;
    }

    bool RRTConnect::newConfig(RRTNode &q, RRTNode &q_near, RRTNode &q_new)
    {
        q_new._configuration.q.clear();
        q_new._parent = -1;

        double dist = 0;
        double step = 0.05;

        for (int i = 0; i < _dimensions; i++)
            dist = dist + std::pow(q._configuration.q.at(i) - q_near._configuration.q.at(i), 2);

        if (step > dist)
            step = dist;

        for (int i = 0; i < _dimensions; i++)
        {
            double step_q;

            step_q = q._configuration.q.at(i) - q_near._configuration.q.at(i);
            step_q = step_q * step / dist;
            q_new._configuration.q.push_back(q_near._configuration.q.at(i) + step_q);
        }

        if (isInCollision(q_new) == false)
            return true;
        else
            return false;
    }

    void RRTConnect::shortcutSmoothing()
    {
        int max_iterations = 40000;
        int rand_1, rand_2;

        _smoothed_path.clear();
        _smoothed_path = _solution_path;

        for (int i = 0; i < max_iterations; i++)
        {
            rand_1 = std::rand() % _smoothed_path.size();
            rand_2 = std::rand() % _smoothed_path.size();

            if (rand_1 == rand_2)
                continue;

            if (connectConfigurations(_tree._nodes.at(_smoothed_path[rand_1]), _tree._nodes.at(_smoothed_path[rand_2])) == true)
            {
                int cnt = rand_2;
                int fin = rand_1;

                std::cout << i << std::endl;

                if (rand_1 < rand_2)
                {
                    cnt = rand_1;
                    fin = rand_2;
                }

                std::cout << "Size: " << _smoothed_path.size() << " - " << cnt << std::endl;
                _smoothed_path.erase(_smoothed_path.begin() + cnt + 1, _smoothed_path.begin() + fin);
            }
        }
    }

    bool RRTConnect::connectConfigurations(RRTNode n1, RRTNode n2)
    {
        RRTNode n_step;

        if (newConfig(n1, n2, n_step) == true)
        {
            n2 = n_step;
            if (n2 == n1)
                return true;
        }

        return false;
    }

    void RRTConnect::drawNearestNeighbourToGoal()
    {
        static OpenRAVE::GraphHandlePtr handle;
        float                           colour[4] = { 1.0, 0, 0, 1.0 };
        float                           points[3];
        RRTNode                         q_near;

        std::vector<OpenRAVE::dReal>    all_dof_values;
        std::vector<OpenRAVE::dReal>    set_dof_values;
        OpenRAVE::EnvironmentMutex::scoped_lock     lock(_env->GetMutex());
        q_near = nearestNeighbour(_goal);

        _robot->GetDOFValues(all_dof_values);
        for (int i = 0; i < _dimensions; i++)
            set_dof_values.push_back(q_near._configuration.q.at(i));
        _robot->SetActiveDOFValues(set_dof_values);

        OpenRAVE::KinBody::LinkPtr      left_gripper;
        OpenRAVE::Transform             trans_fk_l;

        left_gripper = _robot->GetLink("l_gripper_palm_link");
        trans_fk_l = left_gripper->GetTransform();

        points[0] = trans_fk_l.trans.x;
        points[1] = trans_fk_l.trans.y;
        points[2] = trans_fk_l.trans.z;                

        handle = _env->plot3(points, 3, 3 * sizeof(float), 20.0, colour);
        handle->SetShow(true);

        _robot->SetDOFValues(all_dof_values);
    }

    void RRTConnect::drawNode(Configuration q, int colour_)
    {
        static std::vector<OpenRAVE::GraphHandlePtr>    handles;
        OpenRAVE::GraphHandlePtr                        handle;
        float                                           points[3];
        float                                           colour[4] = { 0, 0, 1.0, 1.0 };

        OpenRAVE::KinBody::LinkPtr      left_gripper;
        OpenRAVE::Transform             trans_fk_l;
        std::vector<OpenRAVE::dReal>    dof_values;

        if (colour_ == 1)
        {
            colour[0] = 1.0;
            colour[2] = 0.0;
        }

        for (int i = 0; i < _dimensions; i++)
            dof_values.push_back(q.q.at(i));

        _robot->SetActiveDOFValues(dof_values);

        left_gripper = _robot->GetLink("l_gripper_palm_link");
        trans_fk_l = left_gripper->GetTransform();

        points[0] = trans_fk_l.trans.x;
        points[1] = trans_fk_l.trans.y;
        points[2] = trans_fk_l.trans.z;

        handle = _env->plot3(points, 3, 3 * sizeof(float), 2.0, colour);
        handle->SetShow(true);
        handles.push_back(handle);
        usleep(10000);
    }

    bool RRTConnect::isInCollision(RRTNode &node)
    {
        static OpenRAVE::CollisionCheckerBasePtr           collision_checker;
        static std::vector<OpenRAVE::KinBodyPtr>           bodies;

        if (_collision_check == false)
            return false;

        OpenRAVE::EnvironmentMutex::scoped_lock     lock(_env->GetMutex());
        std::vector<OpenRAVE::dReal>                dof_values;

        for (int i = 0; i < node._configuration.q.size(); i++)
            dof_values.push_back(node._configuration.q.at(i));

        _robot->SetActiveDOFValues(dof_values);

        if (bodies.size() == 0)
            _env->GetBodies(bodies);

        collision_checker = _env->GetCollisionChecker();
        if (collision_checker->SetCollisionOptions(OpenRAVE::CO_Contacts) == false)
        {
            collision_checker = OpenRAVE::RaveCreateCollisionChecker(_env, "ode");
            collision_checker->SetCollisionOptions(OpenRAVE::CO_Contacts);
            _env->SetCollisionChecker(collision_checker);
        }

        if (_env->CheckStandaloneSelfCollision(_robot) == false)
        {            
            for (int i = 0; i < bodies.size(); i++)
                if (bodies[i] != _robot)
                    if (_env->CheckCollision(_robot, bodies[i]) == true)
                        return true;

            return false;
        }

        return true;
    }
}
