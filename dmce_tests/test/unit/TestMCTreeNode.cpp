#include "dmce_test/common.hpp"

#include "dmce_mcplanner/DisplacementAction.hpp"
#include "dmce_mcplanner/MCTreeNode.hpp"

namespace dmce_test {

	using namespace dmce;

	class TestMCTreeNode : public testing::Test {

	public:
		RobotMap map_;
		pos_t robotPos_;
		MCState state_;
		MCActionPtr root_action_;
		MCParams params_;
		MCNodePtr root_node_;
		// MCAction* child_action_;
		MCNodePtr child_node_ = nullptr;

		TestMCTreeNode()
			: robotPos_(0,0),
			  map_({50,50}, 0.1, {0,0}, "map"),
			  state_{robotPos_, 1, map_},
			  params_(Funcs::getMCParams()),
			  root_action_(std::make_shared<DisplacementAction<Displacement::NONE>>(state_, params_)),
			  root_node_(new MCTreeNode(state_, params_, root_action_))//,
			  // child_action_(new SouthDisplacement()),
			  // child_node_(state_, child_action_, &root_node_)
		{
			if (!root_node_->isExpandable(state_)) {
				throw std::runtime_error("Root node is not expandable!");
			}
			auto res = root_node_->expand(state_, root_node_);
			if (!res.first)
				throw std::runtime_error("[TestMCTreeNode] Failed to expand root node!");
			child_node_ = res.second;
		}

		~TestMCTreeNode() {

		}
	};

	TEST_F(TestMCTreeNode, onAddVisit_GetEstimatedValue_isCorrect) {
		EXPECT_EQ(root_node_->getEstimatedValue(), 0);
		root_node_->addVisit(1);
		EXPECT_EQ(root_node_->getEstimatedValue(), 1);
		EXPECT_EQ(root_node_->getDiscountedVisits(), 1);
		root_node_->addVisit(10);
		EXPECT_FLOAT_EQ(root_node_->getEstimatedValue(), 5.5);
		EXPECT_EQ(root_node_->getDiscountedVisits(), 2);
	}

	TEST_F(TestMCTreeNode, testIsRoot) {
		EXPECT_TRUE(root_node_->isRootNode());
		EXPECT_FALSE(child_node_->isRootNode());
	}

	TEST_F(TestMCTreeNode, onRootNode_getUCB_isZero) {
		EXPECT_EQ(root_node_->getUCB(), 0);
		root_node_->addVisit(1);
		EXPECT_EQ(root_node_->getUCB(), 0);
	}

	TEST_F(TestMCTreeNode, onRootNode_onVisit_getUCB_increases) {
		double cur_ucb = child_node_->getUCB();
		EXPECT_EQ(cur_ucb, 0);

		root_node_->addVisit(1);
		child_node_->addVisit(1);

		double new_ucb = child_node_->getUCB();
		EXPECT_GT(new_ucb, cur_ucb);
		cur_ucb = new_ucb;

		root_node_->addVisit(1);
		new_ucb = child_node_->getUCB();
		EXPECT_GT(new_ucb, cur_ucb);
		cur_ucb = new_ucb;
	}

	TEST_F(TestMCTreeNode, onSingleChild_countDescendants_isCorrect) {
		EXPECT_EQ(child_node_->countDescendants(), 0);
		EXPECT_EQ(root_node_->countDescendants(), 1);
	}

	TEST_F(TestMCTreeNode, onSingleChild_childStartPosition_isCorrect) {
		pos_t childIPos = child_node_->getInitialRobotState().pos;
		pos_t parentFPos = root_node_->getFinalRobotState().pos;
		Funcs::expectVectorEqual(childIPos, parentFPos);
		// EXPECT_DOUBLE_EQ(childIPos.x(), parentFPos.x());
		// EXPECT_DOUBLE_EQ(childIPos.y(), parentFPos.y());
	}

	TEST_F(TestMCTreeNode, onGrandchild_countDescendants_isCorrect) {
		child_node_->expand(state_, child_node_);
		EXPECT_EQ(child_node_->countDescendants(), 1);
		EXPECT_EQ(root_node_->countDescendants(), 2);
	}
}

