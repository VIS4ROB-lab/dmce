#include "dmce_sim/OmplPathfinding.hpp"

namespace dmce {

	OmplPathfinding::OmplPathfinding(const OccupancyMap& map) {
		updateMap(map);
	}

	void OmplPathfinding::updateMap(const OccupancyMap& map) {
		occMap_ = map;
		costMap_ = map.toCostmap();
	}

	std::pair<bool, plan_t> OmplPathfinding::makePath(const pos_t& from, const pos_t& to) {
		if (!occMap_.isInside(from))
			throw std::invalid_argument("Starting position must be inside the map!");

		if (!occMap_.isInside(to))
			throw std::invalid_argument("Goal position must be inside the map!");

		plan_t path{};

		if (isStraightLineValidPlan_(from, to)) {
			path.push_back(posToPose(from));
			path.push_back(posToPose(to));
			return {true, path};
		}

		geometry_msgs::PoseStamped start;
		start.pose.position.x = from.x();
		start.pose.position.y = from.y();
		start.pose.position.z = 0;

		geometry_msgs::PoseStamped goal;
		goal.pose.position.x = to.x();
		goal.pose.position.y = to.y();
		goal.pose.position.z = 0;

		tf2::Quaternion orientation;
		orientation.setRPY(0, 0, 0);
		start.pose.orientation = tf2::toMsg(orientation);
		goal.pose.orientation = tf2::toMsg(orientation);

		start.header.frame_id = "map";
		goal.header.frame_id = "map";

		OmplAdapter planner("planner", &costMap_, "map");
		bool success = planner.makePlan(start, goal, path);

		return {success, path};
	}


	bool OmplPathfinding::isStraightLineValidPlan_(const pos_t& from, const pos_t& to) const {
		for (auto it = occMap_.getLineIterator(from, to); !it.isPastEnd(); ++it)
			if (occMap_.isOccupied(*it))
				return false;
		return true;
	}
}
