#include "openrave/openrave.h"
#include "openrave/plugin.h"
#include "boost/bind.hpp"

class Test : public OpenRAVE::ModuleBase
{
    public:
        Test(OpenRAVE::EnvironmentBasePtr penv);
        ~Test();

        bool NumBodies(std::ostream &sout, std::istream &sinput);
        bool Load(std::ostream &sout, std::istream &sinput);
};

