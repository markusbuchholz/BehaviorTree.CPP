#include "behaviortree_cpp_v3/bt_factory.h"

using namespace BT;

// We want to be able to use this custom type
struct Position2D
{
    double x, y;
};

// It is recommended (or, in some cases, mandatory) to define a template
// specialization of convertFromString that converts a string to Position2D.
namespace BT
{
template <>
inline Position2D convertFromString(StringView str)
{
    printf("Converting string: \"%s\"\n", str.data());

    // real numbers separated by semicolons
    auto parts = splitString(str, ';');
    if (parts.size() != 2)
    {
        throw RuntimeError("invalid input)");
    }
    else
    {
        Position2D output;
        output.x = convertFromString<double>(parts[0]);
        output.y = convertFromString<double>(parts[1]);
        return output;
    }
}
}   // end namespace BT

class CheckEnviroment : public SyncActionNode
{
  public:
    CheckEnviroment(const std::string& name, const NodeConfiguration& config)
      : SyncActionNode(name, config)
    {
    }

    NodeStatus tick() override
    {
        std::cout << "Enviroment is NOT good!";
        return NodeStatus::SUCCESS;
    }

    static PortsList providedPorts()
    {
        return {};
    }
};

class CheckRobot : public SyncActionNode
{
  public:
    CheckRobot(const std::string& name, const NodeConfiguration& config)
      : SyncActionNode(name, config)
    {
    }

    NodeStatus tick() override
    {
        std::cout << "Robot is OK!"
                  << "\n";
        return NodeStatus::SUCCESS;
    }

    static PortsList providedPorts()
    {
        return {};
    }
};

class CalculateGoal : public SyncActionNode
{
  public:
    CalculateGoal(const std::string& name, const NodeConfiguration& config)
      : SyncActionNode(name, config)
    {
    }

    NodeStatus tick() override
    {
        Position2D mygoal = {1.1, 2.3};
        setOutput("goal", mygoal);
        return NodeStatus::SUCCESS;
    }
    static PortsList providedPorts()
    {
        return {OutputPort<Position2D>("goal")};
    }
};

class RobotTarget : public SyncActionNode
{
  public:
    RobotTarget(const std::string& name, const NodeConfiguration& config)
      : SyncActionNode(name, config)
    {
    }

    NodeStatus tick() override
    {
        auto res = getInput<Position2D>("input");
        if (!res)
        {
            throw RuntimeError("error reading port [input]:", res.error());
        }
        Position2D goal = res.value();
        printf("Target positions: [ %.1f, %.1f ]\n", goal.x, goal.y);
        return NodeStatus::SUCCESS;
    }

    static PortsList providedPorts()
    {
        // Optionally, a port can have a human readable description
        const char* description = "Simply print the target on console...";
        return {InputPort<Position2D>("input", description)};
    }
};

static const char* xml_text = R"(

 <root main_tree_to_execute = "MainTree" >
     <BehaviorTree ID="MainTree">
        <SequenceStar name="root">
            <CheckRobot   name=" Robot OK"/>
            <CalculateGoal   goal="{Interface}" />
            <RobotTarget     input="{Interface}" />
            <SetBlackboard   output_key="Goal1" value="-1;3" />
            <SetBlackboard   output_key="Goal2" value="5.2;8" />
            <SetBlackboard   output_key="Goal3" value="9;9" />
            <SetBlackboard   output_key="Interface" value="29;9" />
            <RobotTarget     input="{Goal2}" />
            <RobotTarget     input="{Interface}" />
        </SequenceStar>
     </BehaviorTree>
 </root>
 )";

int main()
{
    using namespace BT;

    BehaviorTreeFactory factory;

    factory.registerNodeType<CheckRobot>("CheckRobot");
    factory.registerNodeType<CalculateGoal>("CalculateGoal");
    factory.registerNodeType<RobotTarget>("RobotTarget");

    auto tree = factory.createTreeFromText(xml_text);
    tree.tickRoot();

    return 0;
}
