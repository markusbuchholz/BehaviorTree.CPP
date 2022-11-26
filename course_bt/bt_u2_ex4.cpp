#include "behaviortree_cpp_v3/bt_factory.h"

using namespace BT;

class EatSandwich : public BT::SyncActionNode
{
  public:
    EatSandwich(const std::string& name) : BT::SyncActionNode(name, {})
    {
    }

    // You must override the virtual function tick()
    NodeStatus tick() override
    {
        std::cout << "EatSandwich: " << this->name() << std::endl;
        return BT::NodeStatus::FAILURE;
    }
};

class EatApple : public BT::SyncActionNode
{
  public:
    EatApple(const std::string& name) : BT::SyncActionNode(name, {})
    {
    }

    // You must override the virtual function tick()
    NodeStatus tick() override
    {
        std::cout << "EatApple: " << this->name() << std::endl;
        return BT::NodeStatus::FAILURE;
    }
};

class OpenBanana : public BT::SyncActionNode
{
  public:
    OpenBanana(const std::string& name) : BT::SyncActionNode(name, {})
    {
    }

    // You must override the virtual function tick()
    NodeStatus tick() override
    {
        std::cout << "OpenBanana: " << this->name() << std::endl;
        return BT::NodeStatus::SUCCESS;
    }
};

class EatAction : public BT::AsyncActionNode
{
  public:
    // Any TreeNode with ports must have a constructor with this signature
    EatAction(const std::string& name, const BT::NodeConfiguration& config)
      : AsyncActionNode(name, config)
    {
    }

    // It is mandatory to define this static method.
    static BT::PortsList providedPorts()
    {
        return {};
    }

    BT::NodeStatus tick() override
    {
        printf("[ EatBanana: STARTED ]");

        _halt_requested.store(false);
        int count = 0;

        // Pretend that "computing" takes 250 milliseconds.
        // It is up to you to check periodically _halt_requested and interrupt
        // this tick() if it is true.
        while (!_halt_requested && count++ < 25)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }

        std::cout << "[ EatBanana: FINISHED ]" << std::endl;
        return _halt_requested ? BT::NodeStatus::FAILURE : BT::NodeStatus::SUCCESS;
    }

    virtual void halt() override;

  private:
    std::atomic_bool _halt_requested;
};

void EatAction::halt()
{
    _halt_requested.store(true);
}

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

static const char* xml_text_reactive = R"(

 <root main_tree_to_execute = "MainTree" >

     <BehaviorTree ID="MainTree">
        <ReactiveFallback name="root">
            <EatSandwich name="eat_sandwich"/>
            <EatApple name="eat_apple"/>
            <Sequence>
                <OpenBanana name="open_banana"/>
                <EatBanana  name="eat_banana"/>
                <SaySomething   message="mission completed!" />
            </Sequence>
        </ReactiveFallback>
     </BehaviorTree>

 </root>
 )";

// clang-format on

void Assert(bool condition)
{
    if (!condition)
        throw RuntimeError("this is not what I expected");
}

int main()
{
    using std::chrono::milliseconds;

    BehaviorTreeFactory factory;

    factory.registerNodeType<EatSandwich>("EatSandwich");
    factory.registerNodeType<EatApple>("EatApple");
    factory.registerNodeType<OpenBanana>("OpenBanana");
    factory.registerNodeType<EatAction>("EatBanana");
    factory.registerNodeType<SaySomething>("SaySomething");

    std::cout << "\n------------ BUILDING A NEW TREE ------------" << std::endl;

    auto tree = factory.createTreeFromText(xml_text_reactive);

    NodeStatus status;

    std::cout << "\n--- 1st executeTick() ---" << std::endl;
    status = tree.tickRoot();
    Assert(status == NodeStatus::RUNNING);

    std::this_thread::sleep_for(std::chrono::milliseconds(150));
    std::cout << "\n--- 2nd executeTick() ---" << std::endl;
    status = tree.tickRoot();
    Assert(status == NodeStatus::RUNNING);

    std::this_thread::sleep_for(std::chrono::milliseconds(150));
    std::cout << "\n--- 3rd executeTick() ---" << std::endl;
    status = tree.tickRoot();
    Assert(status == NodeStatus::SUCCESS);

    std::cout << std::endl;

    // }

    return 0;
}
