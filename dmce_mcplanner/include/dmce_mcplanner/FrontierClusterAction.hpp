#pragma once

#include "dmce_mcplanner/MCAction.hpp"

namespace dmce {
	/**
	 * This represents a displacement towards the centroid of
	 * the <clusterIndex>-closest frontier cluster.
	 */
	template<unsigned int clusterIndex>
	class FrontierClusterAction : public MCAction {
		bool isFeasibleFlag_ = true;
		MCRobotState finalRobotState_;

	public:

		/**
		 * Ignores otherActions and parent; sorts the clusters based on distance.
		 */
		FrontierClusterAction(
			const MCState& initialState,
			const MCParams& params,
			const std::vector<MCActionPtr>& otherActions = {},
			const MCActionPtr parent = nullptr
		) : FrontierClusterAction(
				initialState,
				params,
				initialState.clusters.sortClustersByDistance(initialState.robot.pos)
			)
		{ }

		/**
		 * Expects a list of sorted clusters,
		 * as returned by FrontierClustering::sortClustersByDistance.
		 */
		FrontierClusterAction(
			const MCState& initialState, const MCParams& params,
			std::vector<std::pair<unsigned int, double>> sortedClusters
		) : MCAction(initialState, params)
		{
			if (sortedClusters.size() != initialState.clusters.getNClusters())
				throw std::invalid_argument(
						"Number of clusters does not match length of sorted clusters list!"
					);

			if (clusterIndex >= sortedClusters.size()) {
				isFeasibleFlag_ = false;
				return;
			}

			finalRobotState_ = initialState.robot;
			auto centroidPositions = initialState.clusters.getCentroidPositions();
			finalRobotState_.pos = centroidPositions[sortedClusters.at(clusterIndex).first];
		}

		MCRobotState getFinalRobotState() const override {
			return finalRobotState_;
		}

		std::string toString() const override {
			std::stringstream ss;
			ss << "FrontierClusterAction<" << clusterIndex << ">";
			return ss.str();
		}

	protected:
		bool isFeasible_(const MCState& state) const override {
			return isFeasibleFlag_ && !state.map.isOccupied(finalRobotState_.pos);
		}
	};
}
