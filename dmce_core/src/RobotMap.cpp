#include "dmce_core/RobotMap.hpp"
#include "grid_map_core/iterators/CircleIterator.hpp"

#include <ros/ros.h>

namespace dmce {
	void RobotMap::setOccupancy(const grid_map::Position& pos, const float& value) {
		grid_map::Index idx;
		map_.getIndex(pos, idx);
		setOccupancy(idx, value);
	}

	void RobotMap::setOccupancyConservative(
			const grid_map::Position& pos,
			const float& value
	) {
		grid_map::Index idx;
		map_.getIndex(pos, idx);
		setOccupancyConservative(idx, value);
	}

	void RobotMap::setOccupancyConservative(
			const grid_map::Index& idx,
			const float& value
	) {
		if (!isKnown(idx) || value > occupiedThreshold_) {
			setOccupancy(idx, value);
		}
	}

	void RobotMap::setOccupancy(const grid_map::Index& idx, const float& value) {
		if (value < freeValue || value > occupiedValue) {
			std::stringstream ss;
			ss << "Occupancy values must be in range [";
			ss << freeValue << ";" << occupiedValue << "], got " << value;
			throw std::invalid_argument(ss.str());
		}

		bool was_known = isKnown(idx);
		map_.at("occupancy", idx) = value;

		bool is_known = isKnown(idx);
		if (!was_known && is_known)
			--nUnknownCells_;
		else if (was_known && !is_known)
			++nUnknownCells_;
	}
	
	void RobotMap::setOccupancy(
			const std::vector<double>& x,
			const std::vector<double>& y,
			const std::vector<float>& values,
			const unsigned int n)
	{
		for (uint i = 0; i < n; ++i) {
			grid_map::Position pos{x[i], y[i]};
			if (!map_.isInside(pos))
				continue;
			setOccupancy(pos, values[i]);
		}
	}
	
	void RobotMap::setOccupancyConservative(
			const std::vector<double>& x,
			const std::vector<double>& y,
			const std::vector<float>& values,
			const unsigned int n)
	{
		for (uint i = 0; i < n; ++i) {
			grid_map::Position pos{x[i], y[i]};
			if (!map_.isInside(pos))
				continue;
			setOccupancyConservative(pos, values[i]);
		}
	}

	void RobotMap::merge(const RobotMap& other) {
		pos_t pos;
		for (auto it = other.getGridMapIterator(); !it.isPastEnd(); ++it) {
			other.getPosition(*it, pos);
			if (!isInside(pos))
				continue;

			if (!isKnown(pos)) {
				setOccupancy(pos, other.getOccupancy(*it));
			} else if (isFree(pos) && other.isOccupied(*it)) {
				setOccupancy(pos, other.getOccupancy(*it));
			}
		}
	}
};

