#pragma once

namespace mre_dmcts {
	/**
	 * Structure containing the parameters of the MCTS planner.
	 *
	 * Note that int fields are not unsigned to allow for use with ros::param::get.
	 */
	struct MCParams {
		bool reuseBranches;
		int rolloutDepth;
		int minRollouts;
		int minPlanDepth;
		double minPlanValue;
		double explorationFactor;
		double iterationDiscountFactor;
		int robotLidarRayCount;
		double robotSensorRange;
		double robotSpeed;
		bool useActionCaching;
		bool useLocalReward;
		double spatialHashBinSize;
		double navigationCutoff;
		double randomDisplacementMaxTurnAngle;
		double randomDisplacementMinSpread;
		double randomDisplacementLength;
		int randomDisplacementBranchingFactor;
		int planBufferSize;
		double timeDiscountFactor;
		double actionBaseDuration;
		int frontierClusterBranchingFactor;
	};
}
