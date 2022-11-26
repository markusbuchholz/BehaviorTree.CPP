#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/loggers/bt_file_logger.h"

using namespace BT;

class ApproachObject : public BT::SyncActionNode
{
  public:
    ApproachObject(const std::string& name) : BT::SyncActionNode(name, {})
    {
    }

    // You must override the virtual function tick()
    NodeStatus tick() override

    {
        std::cout << "ApproachObject: " << this->name() << std::endl;
        return BT::NodeStatus::SUCCESS;
    }
};

class CheckBattery : public BT::SyncActionNode
{
  public:
    CheckBattery(const std::string& name) : BT::SyncActionNode(name, {})
    {
    }

    // You must override the virtual function tick()
    NodeStatus tick() override
    {
        std::cout << "[ Battery: OK ]" << std::endl;
        return BT::NodeStatus::SUCCESS;
    }
};

class GripperInterface
{
  private:
    bool _opened;

  public:
    GripperInterface() : _opened(true)
    {
    }

    NodeStatus open()
    {
        _opened = true;
        std::cout << "GripperInterface::open" << std::endl;
        return BT::NodeStatus::SUCCESS;
    }

    NodeStatus close()
    {
        std::cout << "GripperInterface::close" << std::endl;
        _opened = false;
        return BT::NodeStatus::SUCCESS;
    }
};

static const char* xml_text = R"(

 <root main_tree_to_execute = "MainTree" >

     <BehaviorTree ID="MainTree">
        <Sequence name="root_sequence">
            <CheckBattery   name="battery_ok"/>
            <ApproachObject name="approach_object"/>
            <OpenGripper    name="open_gripper"/>
        </Sequence>
     </BehaviorTree>

 </root>
 )";

int main()
{
    BehaviorTreeFactory factory;

    factory.registerNodeType<ApproachObject>("ApproachObject");
    factory.registerNodeType<CheckBattery>("CheckBattery");

    GripperInterface gripper;
    factory.registerSimpleAction("OpenGripper", std::bind(&GripperInterface::open, &gripper));

    auto tree = factory.createTreeFromText(xml_text);

    FileLogger logger_file(tree, "bt_trace_u4_ex1.fbl");
    tree.tickRoot();

    return 0;
}
