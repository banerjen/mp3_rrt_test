//#include "openrave/openrave.h"
//#include "openrave/plugin.h"
//#include "boost/bind.hpp"

//using namespace std;
//using namespace OpenRAVE;

//namespace cppexamples{

//class MyModule : public ModuleBase
//{
//public:
//    MyModule(EnvironmentBasePtr penv) : ModuleBase(penv)
//    {
//        __description = "A very simple plugin.";
//        RegisterCommand("numbodies", boost::bind(&MyModule::NumBodies, this, _1, _2), "returns bodies");
//        RegisterCommand("load", boost::bind(&MyModule::Load, this, _1, _2), "loads a given file");
//    }

//    bool NumBodies(ostream& sout, istream& sinput)
//    {
//        vector<KinBodyPtr> vbodies;
//        GetEnv()->GetBodies(vbodies);
//        sout << vbodies.size();
//        return true;
//    }

//    bool Load(ostream& sout, istream& sinput)
//    {
//        string filename;
//        sinput >> filename;
//        bool bSuccess = GetEnv()->Load(filename.c_str());
//        return bSuccess;
//    }
//};
//}


//InterfaceBasePtr CreateInterfaceValidated(InterfaceType type, const std::string& interfacename, std::istream& sinput, EnvironmentBasePtr penv)
//{
//    if( type == PT_Module && interfacename == "mymodule" ) {
//        return InterfaceBasePtr(new cppexamples::MyModule(penv));
//    }
//}

//void GetPluginAttributesValidated(PLUGININFO& info)
//{
//    info.interfacenames[PT_Module].push_back("MyModule");
//}

#include "plugin_test.h"

Test::Test(OpenRAVE::EnvironmentBasePtr penv):ModuleBase(penv)
{
    __description = "A very simple plugin";
    RegisterCommand("numbodies", boost::bind(&Test::NumBodies, this, _1, _2), "returns bodies");
    RegisterCommand("load", boost::bind(&Test::Load, this, _1, _2), "loads a given file");
}

Test::~Test()
{

}

bool Test::NumBodies(std::ostream &sout, std::istream &sinput)
{
    std::vector<OpenRAVE::KinBodyPtr>   vbodies;
    GetEnv()->GetBodies(vbodies);
    sout << vbodies.size();
    return true;
}

bool Test::Load(std::ostream &sout, std::istream &sinput)
{
    std::string filename;
    sinput >> filename;
    bool bSuccess = GetEnv()->Load(filename.c_str());
    return bSuccess;
}

OpenRAVE::InterfaceBasePtr CreateInterfaceValidated(OpenRAVE::InterfaceType type, const std::string &name, std::istream &sinput, OpenRAVE::EnvironmentBasePtr penv)
{
    if (type == OpenRAVE::PT_Module && name == "testmodule")
        return OpenRAVE::InterfaceBasePtr(new Test(penv));
}

void GetPluginAttributesValidated(OpenRAVE::PLUGININFO &info)
{
    info.interfacenames[OpenRAVE::PT_Module].push_back("TestModule");
}
