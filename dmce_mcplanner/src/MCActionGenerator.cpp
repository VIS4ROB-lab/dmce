#include "dmce_mcplanner/MCActionGenerator.hpp"

namespace dmce {
	const MCActionGenerator::gen_map_t MCActionGenerator::generator_map = { };

	MCActionGenerator::cache_map_t MCActionGenerator::actionCache_;



	std::vector<MCActionPtr> MCActionGenerator::generateFeasibleActions(
			const MCState& initialState,
			const MCParams& params,
			const MCNodePtr parent,
			bool useFrontiers,
			bool useDisplacements
	) {
		MCActionPtr parentAction = nullptr;
		if (parent != nullptr)
			parentAction = parent->getAction();

		std::vector<MCActionPtr> feasibleActions;

		if (useDisplacements)
			addRandomDisplacementActions_(initialState, params, parentAction, feasibleActions);

		if (useFrontiers)
			addFrontierClusterActions_(initialState, params, parentAction, feasibleActions);

		// Actions from "hard-coded" list this->generator_map
		for (auto kv_pair : generator_map) {
			addAction_(initialState, params, parentAction,
					generator_map.at(kv_pair.first), feasibleActions
				);
		}

		return feasibleActions;
	}


	void MCActionGenerator::addRandomDisplacementActions_(
			const MCState& initialState,
			const MCParams& params,
			const MCActionPtr& parentAction,
			std::vector<MCActionPtr>& actionArray
	) {
		for (size_t i = 0; i < params.randomDisplacementBranchingFactor; i++) {
			addAction_(initialState, params, parentAction,
					&actionFactory<RandomDisplacement>, actionArray
				);
		}
	}


	void MCActionGenerator::addFrontierClusterActions_(
			const MCState& initialState,
			const MCParams& params,
			const MCActionPtr& parentAction,
			std::vector<MCActionPtr>& actionArray
	) {
		unsigned int branchingFactor = params.frontierClusterBranchingFactor;
		if (branchingFactor > 5) {
			branchingFactor = 5;
			ROS_WARN("[MCActionGenerator::addFrontierClusterActions_] Parameter /mcts/frontierClusterAction/branchingFactor too large! Max value is %u.", branchingFactor);
		}

		// Note the use of break statements to intentionally allow fall-through
		switch (params.frontierClusterBranchingFactor) {
			case 5:
				addAction_(initialState, params, parentAction,
						&actionFactory<FrontierClusterAction<4>>, actionArray);
			case 4:
				addAction_(initialState, params, parentAction,
						&actionFactory<FrontierClusterAction<3>>, actionArray);
			case 3:
				addAction_(initialState, params, parentAction,
						&actionFactory<FrontierClusterAction<2>>, actionArray);
			case 2:
				addAction_(initialState, params, parentAction,
						&actionFactory<FrontierClusterAction<1>>, actionArray);
			case 1:
				addAction_(initialState, params, parentAction,
						&actionFactory<FrontierClusterAction<0>>, actionArray);
			default:
				break;
		}
	}


	void MCActionGenerator::addAction_(
			const MCState& initialState,
			const MCParams& params,
			const MCActionPtr& parentAction,
			const factory_ptr_t& factory,
			std::vector<MCActionPtr>& actionArray
	) {
		auto action_ptr = factory(initialState, params, actionArray, parentAction);

		if (params.useActionCaching)
			checkActionCache_(action_ptr);

		if (action_ptr->isFeasible(initialState))
			actionArray.push_back(action_ptr);
	}


	void MCActionGenerator::checkActionCache_(MCActionPtr& action_ptr) {
		if (!action_ptr->isCacheable())
			return;

		auto key = action_ptr->getHash();
		if (actionCache_.find(key) != actionCache_.end()) {
			action_ptr = actionCache_[key];
		} else {
			actionCache_[key] = action_ptr;
		}
	}


}
