#include "behaviortree_cpp_v3/bt_factory.h"

using namespace BT;

class Obstacle : public BT::SyncActionNode
{
  public:
    Obstacle(const std::string& name) : BT::SyncActionNode(name, {})
    {
    }

    // You must override the virtual function tick()
    NodeStatus tick() override
    {
        std::cout << "OK. Detected !"
                  << "\n";
        return BT::NodeStatus::SUCCESS;
    }
};

class EnoughBattery : public BT::SyncActionNode
{
  public:
    EnoughBattery(const std::string& name) : BT::SyncActionNode(name, {})
    {
    }

    // You must override the virtual function tick()
    NodeStatus tick() override
    {
        std::cout << "is there energy remain?: " << this->name() << std::endl;
        return BT::NodeStatus::SUCCESS;
    }
};

class Rotate : public BT::SyncActionNode
{
  public:
    Rotate(const std::string& name) : BT::SyncActionNode(name, {})
    {
    }

    // You must override the virtual function tick()
    NodeStatus tick() override
    {
        std::cout << "rotating 90 deg : " << this->name() << std::endl;
        return BT::NodeStatus::SUCCESS;
    }
};

class MoveStraight : public BT::SyncActionNode
{
  public:
    MoveStraight(const std::string& name) : BT::SyncActionNode(name, {})
    {
    }

    // You must override the virtual function tick()
    NodeStatus tick() override
    {
        std::cout << "moving again straight : " << this->name() << std::endl;
        return BT::NodeStatus::SUCCESS;
    }
};

static const char* xml_text = R"(

 <root main_tree_to_execute = "MainTree" >

     <BehaviorTree ID="MainTree">
        <Sequence name="root_sequence">
            <Obstacle   name=" Obstacle detected"/>
            <EnoughBattery   name=" Battery OK"/>
            <Rotate       name=" Rotated"/>
            <MoveStraight name=" Moved"/>
        </Sequence>
     </BehaviorTree>

 </root>
 )";

int main()
{
    // We use the BehaviorTreeFactory to register our custom nodes
    BehaviorTreeFactory factory;

    // The recommended way to create a Node is through inheritance.
    // Even if it requires more boilerplate, it allows you to use more functionalities
    // like ports (we will discuss this in future tutorials).

    factory.registerNodeType<Obstacle>("Obstacle");
    factory.registerNodeType<EnoughBattery>("EnoughBattery");
    factory.registerNodeType<Rotate>("Rotate");
    factory.registerNodeType<MoveStraight>("MoveStraight");

    // Trees are created at deployment-time (i.e. at run-time, but only once at the beginning).
    // The currently supported format is XML.
    // IMPORTANT: when the object "tree" goes out of scope, all the TreeNodes are destroyed
    auto tree = factory.createTreeFromText(xml_text);

    // To "execute" a Tree you need to "tick" it.
    // The tick is propagated to the children based on the logic of the tree.
    // In this case, the entire sequence is executed, because all the children
    // of the Sequence return SUCCESS.
    tree.tickRoot();

    return 0;
}
