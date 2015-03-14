//#include "ros/ros.h"

#include "cstdlib"
#include "ctime"
#include "iostream"
#include "vector"
#include "iterator"
#include "list"
#include "algorithm"
#include "chrono"

#include "openrave-core.h"
//#include "openrave/plugin.h"
#include "boost/bind.hpp"

namespace nbRRT
{
    typedef double QValue;

    enum class ExtendModes { Reached, Advanced, Trapped, Solution };

    class Configuration
    {
        public:
            std::vector<QValue> q;            

            Configuration();
            ~Configuration();
            void operator =(const Configuration &C);
            bool operator ==(const Configuration &C);
    };

    class RRTNode
    {
        public:
            Configuration           _configuration;
            int                     _index;
            int                     _parent;

            RRTNode();
            ~RRTNode();
            void operator =(const RRTNode &N);
            bool operator ==(const RRTNode &N);
    };

    class NodeTree
    {
        public:
            std::vector<RRTNode>    _nodes;
            bool                    _initialised;

            NodeTree();
            ~NodeTree();
            void init(RRTNode q);
            bool addVertex(RRTNode q);
            bool addEdge(RRTNode q1, RRTNode q2);
            void getPath(RRTNode from_node, std::vector<int> &indices);
    };

    class RRTConnect// : public OpenRAVE::ModuleBase
    {
        public:
            RRTConnect();
            ~RRTConnect();
            void setInit(Configuration &q_init);
            void setGoal(Configuration &q_goal);            
            void setStepSize(std::vector<QValue> &step_size);
            void setGoalBias(int bias);
            void setCollisionCheckFlag(bool enable);
            void setDOFLimits(std::vector<QValue> &upper_limits, std::vector<QValue> &lower_limits);
            void setEnvironment(OpenRAVE::EnvironmentBasePtr &env);
            void setRobot(OpenRAVE::RobotBasePtr &robot);
            void setTimeLimit(int seconds);
            void setJointIndices(std::vector<int> &joint_indices);
            bool findPath(std::vector<Configuration> &solution);

        private:
            OpenRAVE::EnvironmentBasePtr    _env;
            NodeTree                        _tree;
            RRTNode                         _init;
            RRTNode                         _goal;            
            bool                            _collision_check;
            std::vector<QValue>             _q_upper_limits;
            std::vector<QValue>             _q_lower_limits;
            std::vector<std::string>        _joint_names;
            std::vector<int>                _joint_indices;
            double                          _dimensions;
            int                             _goal_bias;
            std::vector<QValue>             _step_size;
            int                             _seconds;
            bool                            _result;
            std::vector<int>                _solution_path;
            std::vector<int>                _smoothed_path;
            OpenRAVE::RobotBasePtr          _robot;

            // Main RRT functions
            void buildRRT(RRTNode q_init);
            ExtendModes extend(RRTNode &q);
            ExtendModes connect(RRTNode &q);

            // Helper functions
            void randomConfig(RRTNode &q_rand);
            RRTNode nearestNeighbour(RRTNode &q);
            bool newConfig(RRTNode &q, RRTNode &q_near, RRTNode &q_new);
            void shortcutSmoothing();
            bool connectConfigurations(RRTNode n1, RRTNode n2);

            // Drawing
            void drawNode(Configuration q, int colour);
            void drawNearestNeighbourToGoal();            

            // Collision detection
            bool isInCollision(RRTNode &node);
    };
}
