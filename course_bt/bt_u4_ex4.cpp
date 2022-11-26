#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/loggers/bt_file_logger.h"

using namespace BT;

struct Range
{
    int low;
    int high;
};

class Sensor : public SyncActionNode
{
  private:
    int _arg1;
    int _arg2;

  public:
    // additional arguments passed to the constructor
    Sensor(const std::string& name, const NodeConfiguration& config, int arg1, int arg2)
      : SyncActionNode(name, config), _arg1(arg1), _arg2(arg2)
    {
    }

    NodeStatus tick() override
    {
        Range range = {range.low = _arg1, range.high = _arg2};

        std::cout << "sensor range low : " << range.low << " high :" << range.high << "\n";
        setOutput("range", range);

        return NodeStatus::SUCCESS;
    }
    static PortsList providedPorts()
    {
        return {OutputPort<Range>("range")};
    }
};

class RobotBrain
{
  private:
    float result;

  public:
    RobotBrain(){};

    float compute(float f1, float f2, float f3)
    {
        return (f1 * f1 + f2 * 0.43 + f3 * 3.1415);
    }
};

class Compute : public SyncActionNode, public RobotBrain
{
  private:
    float _arg1;
    float _arg2;
    float _arg3;

  public:
    Compute(const std::string& name, const NodeConfiguration& config) : SyncActionNode(name, config)
    {
    }

    void init(float arg1, float arg2, float arg3)
    {
        _arg1 = (arg1);
        _arg2 = (arg2);
        _arg3 = (arg3);
    }

    NodeStatus tick() override
    {
        auto res = getInput<Range>("brain_input");
        if (!res)
        {
            throw RuntimeError("FROM BRAIN error reading port [input]:", res.error());
        }
        auto range = res.value();

        std::cout << "Compute init factors: " << _arg1 << " : " << _arg2 << " : " << _arg3
                  << std::endl;

        RobotBrain brain;
        float result = brain.compute(_arg1 * range.low, _arg2 * range.high, _arg3);
        setOutput("brain_output", result);

        return NodeStatus::SUCCESS;
    }

    static PortsList providedPorts()
    {
        return {InputPort<Range>("brain_input"), OutputPort<float>("brain_output")};
    }
};

class AnalyzeMeasurement : public SyncActionNode
{
  public:
    AnalyzeMeasurement(const std::string& name, const NodeConfiguration& config)
      : SyncActionNode(name, config)
    {
    }

    NodeStatus tick() override
    {
        auto res = getInput<float>("input");
        if (!res)
        {
            throw RuntimeError("error reading port [input]:", res.error());
        }
        auto meas = res.value();

        std::cout << " Measurements ::: " << meas << "\n";

        std::cout << "[ Analysis process: FINISHED ]" << std::endl;

        return NodeStatus::SUCCESS;
    }

    static PortsList providedPorts()
    {
        // Optionally, a port can have a human readable description
        const char* description = "Simply print cpu usage on terminal...";
        return {InputPort<float>("input")};
    }
};

static const char* xml_text = R"(

 <root >
     <BehaviorTree>
        <Sequence>
            <Sensor range="{range}"/>
            <Compute brain_input="{range}" brain_output="{meas}"/>
             <AnalyzeMeasurement  input="{meas}"/>
            
            
        </Sequence>
     </BehaviorTree>
 </root>
 )";

int main()
{
    BehaviorTreeFactory factory;

    NodeBuilder builder_A = [](const std::string& name, const NodeConfiguration& config) {
        return std::make_unique<Sensor>(name, config, 10, 30);
    };

    factory.registerBuilder<Sensor>("Sensor", builder_A);

    factory.registerNodeType<Compute>("Compute");

    factory.registerNodeType<AnalyzeMeasurement>("AnalyzeMeasurement");

    auto tree = factory.createTreeFromText(xml_text);

    FileLogger logger_file(tree, "bt_trace_u3_ex4.fbl");

    for (auto& node : tree.nodes)
    {
        if (auto Compute_node = dynamic_cast<Compute*>(node.get()))
        {
            Compute_node->init(1.1, 1.2, 1.3);
        }
    }

    tree.tickRoot();

    return 0;
}
