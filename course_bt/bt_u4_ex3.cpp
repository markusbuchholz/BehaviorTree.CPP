#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/loggers/bt_file_logger.h"
#include <thread>
#include <chrono>

using namespace BT;

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
        return (c * load * 1);
    }
};

class ApproachObject : public BT::SyncActionNode
{
  public:
    ApproachObject(const std::string& name) : BT::SyncActionNode(name, {})
    {
    }

    NodeStatus tick() override
    {
        std::cout << "ApproachObject: " << this->name() << std::endl;
        return BT::NodeStatus::SUCCESS;
    }
};

class AnalyzeBattery : public BT::SyncActionNode
{
  public:
    AnalyzeBattery(const std::string& name) : BT::SyncActionNode(name, {})
    {
    }

    NodeStatus tick() override
    {
        std::cout << "Analyze Battery: " << this->name() << std::endl;
        return BT::NodeStatus::SUCCESS;
    }
};

class GetCPUusage : public SyncActionNode, public ComputeUsage
{
    static int count;

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
        count += 1;
        ComputeUsage cpu;
        auto cpu_load = cpu.calulateUsege(1, 3.1);
        this->usage = cpu_load;
        CPU cpuX = {cpu_load};
        std::cout << "current COUNTER ===> : " << count << "\n";

        std::cout << "[ getting: FINISHED ]" << std::endl;
        setOutput("usage", cpuX);

        return NodeStatus::SUCCESS;
    }
    static PortsList providedPorts()
    {
        return {OutputPort<CPU>("usage")};
    }
};

class AnalyzeCPU : public AsyncActionNode
{
  private:
    std::atomic_bool _halt_requested;

  public:
    AnalyzeCPU(const std::string& name, const NodeConfiguration& config)
      : AsyncActionNode(name, config)
    {
    }

    virtual void halt() override;

    NodeStatus tick() override
    {
        auto res = getInput<CPU>("input");
        if (!res)
        {
            throw RuntimeError("error reading port [input]:", res.error());
        }
        CPU usage = res.value();
        _halt_requested.store(false);

        int count = 0;

        while ((!_halt_requested) && (count < 10))
        {
            std::cout << "analyzing for :: " << count << "\n";
            std::this_thread::sleep_for(std::chrono::seconds(1));
            count++;
        }

        printf("Analyze CPU ::: [ %.1f ]\n", usage.cpu);
        //return NodeStatus::SUCCESS;

        std::cout << "[ Analysis process: FINISHED ]" << std::endl;

        return _halt_requested ? BT::NodeStatus::FAILURE : BT::NodeStatus::SUCCESS;
    }

    static PortsList providedPorts()
    {
        // Optionally, a port can have a human readable description
        const char* description = "Simply print cpu usage on terminal...";
        return {InputPort<CPU>("input", description)};
    }
};

void AnalyzeCPU::halt()
{
    _halt_requested.store(false);

}



static const char* xml_text = R"(

 <root main_tree_to_execute = "MainTree" >

     <BehaviorTree ID="MainTree">
        <ReactiveSequence name="root_sequence">
            <AnalyzeBattery   name="battery_ok"/>   
            <GetCPUusage   usage="{CPUload}" />
            <AnalyzeCPU   input="{CPUload}" />
        </ReactiveSequence>
     </BehaviorTree>

 </root>
 )";

int GetCPUusage::count{0};
int main()
{
    BehaviorTreeFactory factory;
    using std::chrono::milliseconds;

    factory.registerNodeType<AnalyzeBattery>("AnalyzeBattery");
    factory.registerNodeType<GetCPUusage>("GetCPUusage");
    factory.registerNodeType<AnalyzeCPU>("AnalyzeCPU");

    auto tree = factory.createTreeFromText(xml_text);

    FileLogger logger_file(tree, "bt_trace_u3_ex3.fbl");

    std::cout << "\n--- 1 executeTick() ---" << std::endl;
    tree.tickRoot();
    std::this_thread::sleep_for(std::chrono::microseconds(1000));

    std::cout << "\n--- 2 executeTick() ---" << std::endl;
    tree.tickRoot();
    std::this_thread::sleep_for(std::chrono::microseconds(1000));

    std::cout << "\n--- 3 executeTick() ---" << std::endl;
    tree.tickRoot();
    std::this_thread::sleep_for(std::chrono::microseconds(1000));

    std::cout << "\n--- 4 executeTick() ---" << std::endl;
    tree.tickRoot();
    std::this_thread::sleep_for(std::chrono::microseconds(1000));

    return 0;
}
