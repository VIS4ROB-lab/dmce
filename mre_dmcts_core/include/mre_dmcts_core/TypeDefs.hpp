#pragma once

#include <mutex>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "grid_map_core/TypeDefs.hpp"

namespace mre_dmcts {
	using pos_t = grid_map::Position;
	using pose_t = geometry_msgs::PoseStamped;
	using plan_t = std::vector<pose_t>;

	using mutex_t = std::mutex;
	using lockGuard_t = std::lock_guard<mutex_t>;

	using index_t = grid_map::Index;
	using indexList_t = std::vector<index_t>;

	/**
	 * Convert a 2D position to a 3D pose
	 *  with z=0 and orientation (0,0,0).
	 */
	inline pose_t posToPose(const pos_t& position) {
		pose_t poseStamped;
		poseStamped.pose.position.x = position.x();
		poseStamped.pose.position.y = position.y();
		poseStamped.pose.position.z = 0;
		tf2::Quaternion orientation;
		orientation.setRPY(0, 0, 0);
		poseStamped.pose.orientation = tf2::toMsg(orientation);
		return poseStamped;
	}

	inline pos_t poseToPos(const pose_t& pose) {
		pos_t position;
		position.x() = pose.pose.position.x;
		position.y() = pose.pose.position.y;
		return position;
	}
}

// Specialize std::less for the intex type
namespace std {
	template<>
	struct less<mre_dmcts::index_t>
	{
	   bool operator()(const mre_dmcts::index_t& k1, const mre_dmcts::index_t& k2) const
	   {
			// Lexicographic comparison k1 < k2
			if (k1.x() != k2.x())
				return k1.x() < k2.x();
			return k1.y() < k2.y();
	   }
	};
}

