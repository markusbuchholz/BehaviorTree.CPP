#include "behaviortree_cpp_v3/loggers/bt_cout_logger.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/loggers/bt_file_logger.h"
#include <thread>
#include <chrono>

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

class SaySomething : public BT::SyncActionNode
{
  public:
    SaySomething(const std::string& name, const BT::NodeConfiguration& config)
      : BT::SyncActionNode(name, config)
    {
    }

    // You must override the virtual function tick()
    NodeStatus tick() override
    {
        auto msg = getInput<std::string>("message");
        if (!msg)
        {
            throw BT::RuntimeError("missing required input [message]: ", msg.error());
        }

        std::cout << "Robot says: " << msg.value() << std::endl;
        return BT::NodeStatus::SUCCESS;
    }

    // It is mandatory to define this static method.
    static BT::PortsList providedPorts()
    {
        return {BT::InputPort<std::string>("message")};
    }
};

// Custom type
struct Pose2D
{
    double x, y, theta;
};

namespace BT
{

template <>
inline Pose2D convertFromString(StringView key)
{
    // three real numbers separated by semicolons
    auto parts = BT::splitString(key, ';');
    if (parts.size() != 3)
    {
        throw BT::RuntimeError("invalid input)");
    }
    else
    {
        Pose2D output;
        output.x = convertFromString<double>(parts[0]);
        output.y = convertFromString<double>(parts[1]);
        output.theta = convertFromString<double>(parts[2]);
        return output;
    }
}
}   // end namespace BT

class MoveBaseAction : public BT::AsyncActionNode
{
  public:
    // Any TreeNode with ports must have a constructor with this signature
    MoveBaseAction(const std::string& name, const BT::NodeConfiguration& config)
      : AsyncActionNode(name, config)
    {
    }

    // It is mandatory to define this static method.
    static BT::PortsList providedPorts()
    {
        return {BT::InputPort<Pose2D>("goal")};
    }

    BT::NodeStatus tick() override
    {
        Pose2D goal;
        if (!getInput<Pose2D>("goal", goal))
        {
            throw BT::RuntimeError("missing required input [goal]");
        }

        printf("[ MoveBase: STARTED ]. goal: x=%.f y=%.1f theta=%.2f\n", goal.x, goal.y,
               goal.theta);

        _halt_requested.store(false);
        int count = 0;

        // Pretend that "computing" takes 250 milliseconds.
        // It is up to you to check periodically _halt_requested and interrupt
        // this tick() if it is true.
        while (!_halt_requested && count++ < 25)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }

        std::cout << "[ MoveBase: FINISHED ]" << std::endl;
        return _halt_requested ? BT::NodeStatus::FAILURE : BT::NodeStatus::SUCCESS;
    }

    virtual void halt() override;

  private:
    std::atomic_bool _halt_requested;
};

void MoveBaseAction::halt()
{
    _halt_requested.store(true);
}

static const char* xml_text = R"(
<root main_tree_to_execute = "MainTree">

    <BehaviorTree ID="MainTree">

        <Sequence name="main_sequence">
            <SetBlackboard output_key="move_goalXY" value="1;2;3" />
            <SubTree ID="MoveRobotXY" targetXY="move_goalXY" outputXY="move_resultXY" />
            <RobotTarget     input="{move_resultXY}" />

            <SetBlackboard output_key="move_goal" value="1;2;3" />
            <SubTree ID="MoveRobot" target="move_goal" output="move_result" />
            <SaySomething message="{move_result}"/>
        </Sequence>

    </BehaviorTree>

    <BehaviorTree ID="MoveRobotXY">
        <Fallback name="move_robot_main">
            <SequenceStar>
                <MoveBase       goal="{targetXY}"/>
                <SetBlackboard output_key="outputXY" value="8;9" />
            </SequenceStar>
            <ForceFailure>
                <SetBlackboard output_key="outputXY" value="0;0" />
            </ForceFailure>
        </Fallback>
    </BehaviorTree>


    <BehaviorTree ID="MoveRobot">
        <Fallback name="move_robot_main">
            <SequenceStar>
                <MoveBase       goal="{target}"/>
                <SetBlackboard output_key="output" value="mission accomplished" />
            </SequenceStar>
            <ForceFailure>
                <SetBlackboard output_key="output" value="mission failed" />
            </ForceFailure>
        </Fallback>
    </BehaviorTree>

</root>
 )";

using namespace BT;

int main()
{
    BehaviorTreeFactory factory;

    factory.registerNodeType<SaySomething>("SaySomething");
    factory.registerNodeType<MoveBaseAction>("MoveBase");
    factory.registerNodeType<RobotTarget>("RobotTarget");

    auto tree = factory.createTreeFromText(xml_text);
    FileLogger logger_file(tree, "bt_trace56.fbl");

    NodeStatus status = NodeStatus::RUNNING;
    // Keep on ticking until you get either a SUCCESS or FAILURE state
    while (status == NodeStatus::RUNNING)
    {
        status = tree.tickRoot();

        std::this_thread::sleep_for(std::chrono::microseconds(1));
    }

    // let's visualize some information about the current state of the blackboards.
    std::cout << "--------------" << std::endl;
    tree.blackboard_stack[0]->debugMessage();
    std::cout << "--------------" << std::endl;
    tree.blackboard_stack[1]->debugMessage();
    std::cout << "--------------" << std::endl;

    return 0;
}
