#pragma once
#include <string>
#include <map>

#include "dmce_mcplanner/MCAction.hpp"
#include "dmce_mcplanner/DisplacementAction.hpp"
#include "dmce_mcplanner/MCTreeNode.hpp"
#include "dmce_mcplanner/FrontierClusterAction.hpp"

namespace dmce {
	class MCActionGenerator {
		template<typename T>
		static MCActionPtr actionFactory(
			const MCState& initialState,
			const MCParams& params,
			const std::vector<MCActionPtr>& otherActions,
			const MCActionPtr parent
		) {
			return std::make_shared<T>(initialState, params, otherActions, parent);
		}

	public:
		using factory_ptr_t =MCActionPtr(*)(const MCState&, const MCParams&, const std::vector<MCActionPtr>&, const MCActionPtr);
		using gen_map_t = std::map<std::string, factory_ptr_t>;
		const static gen_map_t generator_map;
		
		using cache_map_t = std::map<MCAction::hashKey_t, MCActionPtr>;
		static cache_map_t actionCache_;

		MCActionGenerator() { }

		/**
		 * Returns the set of feasible actions given the starting state,
		 * parameters and optional parent node.
		 */
		std::vector<MCActionPtr> generateFeasibleActions(
				const MCState& initialState,
				const MCParams& params,
				const MCNodePtr parent = nullptr,
				bool useFrontiers = false,
				bool useDisplacements = true
		);

	protected:
		/**
		 * Construct an action with the given factory pointer and
		 * add it to the action array if it's feasible.
		 */
		void addAction_(
				const MCState& initialState,
				const MCParams& params,
				const MCActionPtr& parentAction,
				const factory_ptr_t& factory,
				std::vector<MCActionPtr>& actionArray
		);

		/**
		 * Add the frontier-cluster actions to the actionArray.
		 */
		void addFrontierClusterActions_(
				const MCState& initialState,
				const MCParams& params,
				const MCActionPtr& parentAction,
				std::vector<MCActionPtr>& actionArray
		);

		/**
		 * Add the random displacement actions to the actionArray.
		 */
		void addRandomDisplacementActions_(
				const MCState& initialState,
				const MCParams& params,
				const MCActionPtr& parentAction,
				std::vector<MCActionPtr>& actionArray
		);

		/**
		 * If the given action type is cacheable, check if we already have this
		 * action cached. If so, replace action_ptr with the cached copy.
		 * If not, add action_ptr to the action cache.
		 */
		void checkActionCache_(MCActionPtr& action_ptr);
	};
};
