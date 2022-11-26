#include "behaviortree_cpp_v3/bt_factory.h"

using namespace BT;

class ObstacleCheck : public BT::SyncActionNode
{
  public:
    int count;

    ObstacleCheck(const std::string& name) : BT::SyncActionNode(name, {})
    {
        this->count = 0;
    }

    // You must override the virtual function tick()
    NodeStatus tick() override
    {
        std::cout << "OK. Detected !"
                  << "\n";
        return BT::NodeStatus::SUCCESS;
    }
};

class takeDecision : public BT::AsyncActionNode
{
  public:
    // Any TreeNode with ports must have a constructor with this signature
    takeDecision(const std::string& name, const BT::NodeConfiguration& config)
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
        {
            _halt_requested.store(false);

            int count = 0;

            while ((!_halt_requested) && (count < 5))
            {
                std::cout << "thinking for :: " << count << "\n";
                std::this_thread::sleep_for(std::chrono::seconds(1));
                count++;
            }

            std::cout << "[ Decision process: FINISHED ]" << std::endl;

            return _halt_requested ? BT::NodeStatus::FAILURE : BT::NodeStatus::SUCCESS;
        }
    }

    virtual void halt() override;

  private:
    std::atomic_bool _halt_requested;
};

void takeDecision::halt()
{
    _halt_requested.store(false);

}

static const char* xml_text_reactive = R"(

 <root main_tree_to_execute = "MainTree" >

     <BehaviorTree ID="MainTree">
        <ReactiveSequence name="root">
            <ObstacleCheck   name=" Obstacle detected"/>       
                <takeDecision   name="take Decision"/>
        </ReactiveSequence>
     </BehaviorTree>

 </root>
 )";

void Assert(bool condition)
{
    if (!condition)
        throw RuntimeError("this is not what I expected");
}

int main()
{
    using std::chrono::milliseconds;

    BehaviorTreeFactory factory;

    factory.registerNodeType<ObstacleCheck>("ObstacleCheck");
    factory.registerNodeType<takeDecision>("takeDecision");

    std::cout << "\n------------ BUILDING A NEW TREE ------------" << std::endl;

    auto tree = factory.createTreeFromText(xml_text_reactive);

    NodeStatus status;
    status = tree.tickRoot();

    std::cout << "\n--- 1st executeTick() ---" << std::endl;
    status = tree.tickRoot();

    std::this_thread::sleep_for(std::chrono::microseconds(1000));
    std::cout << "\n--- 2nd executeTick() ---" << std::endl;
    status = tree.tickRoot();

    std::this_thread::sleep_for(std::chrono::microseconds(1000));
    std::cout << "\n--- 3rd executeTick() ---" << std::endl;
    status = tree.tickRoot();

    return 0;
}
