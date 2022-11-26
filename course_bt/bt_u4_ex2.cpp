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

struct Position2D
{
    double x, y;
};

struct CPU
{
    float cpu;
};

class ComputeUsage
{
  private:
    int cores;
    float load_pr_core;

  public:
    ComputeUsage()
    {
    }

    float calulateUsege(int c, float load)
    {
        return (c * load * 3.1415);
    }
};

class GetCPUusage : public SyncActionNode, public ComputeUsage
{
  private:
    float usage;

  public:
    GetCPUusage(const std::string& name, const NodeConfiguration& config)
      : SyncActionNode(name, config)
    {
    }

    //node callback
    NodeStatus tick() override
    {
        ComputeUsage cpu;
        auto cpu_load = cpu.calulateUsege(4, 4.8);
        this->usage = cpu_load;
        CPU cpuX = {cpu_load};

        setOutput("usage", cpuX);
        return NodeStatus::SUCCESS;
    }
    static PortsList providedPorts()
    {
        return {OutputPort<CPU>("usage")};
    }
};

class AnalyzeCPU : public SyncActionNode
{
  public:
    AnalyzeCPU(const std::string& name, const NodeConfiguration& config)
      : SyncActionNode(name, config)
    {
    }

    NodeStatus tick() override
    {
        auto res = getInput<CPU>("input");
        if (!res)
        {
            throw RuntimeError("error reading port [input]:", res.error());
        }
        CPU usage = res.value();
        printf("Analyze CPU ::: [ %.1f ]\n", usage.cpu);
        return NodeStatus::SUCCESS;
    }

    static PortsList providedPorts()
    {
        // Optionally, a port can have a human readable description
        const char* description = "Simply print cpu usage on terminal...";
        return {InputPort<CPU>("input", description)};
    }
};

static const char* xml_text = R"(

 <root main_tree_to_execute = "MainTree" >

     <BehaviorTree ID="MainTree">
        <Sequence name="root_sequence">
            <CheckBattery   name="battery_ok"/>
            <ApproachObject name="approach_object"/>
            <OpenGripper    name="open_gripper"/>
            <GetCPUusage   usage="{CPUload}" />
            <AnalyzeCPU   input="{CPUload}" />
        </Sequence>
     </BehaviorTree>

 </root>
 )";

int main()
{
    // We use the BehaviorTreeFactory to register our custom nodes
    BehaviorTreeFactory factory;

    factory.registerNodeType<ApproachObject>("ApproachObject");
    factory.registerNodeType<GetCPUusage>("GetCPUusage");
    factory.registerNodeType<AnalyzeCPU>("AnalyzeCPU");
    factory.registerNodeType<CheckBattery>("CheckBattery");

    GripperInterface gripper;
    factory.registerSimpleAction("OpenGripper", std::bind(&GripperInterface::open, &gripper));

    auto tree = factory.createTreeFromText(xml_text);

    FileLogger logger_file(tree, "bt_trace_u3_ex2.fbl");
    tree.tickRoot();

    return 0;
}
