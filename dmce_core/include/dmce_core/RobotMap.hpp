#pragma once

#include <ros/ros.h>
#include "grid_map_core/GridMap.hpp"
#include "grid_map_ros/grid_map_ros.hpp"

#include "dmce_core/OccupancyMap.hpp"

namespace dmce {

	/**
	 * This class represents a 2D occupancy map held by an individual robot.
	 */
	class RobotMap : public OccupancyMap {
		
	public:

		// Inherit constructors
		using OccupancyMap::OccupancyMap;

		/**
		 * Set the occupancy value at the given position.
		 * Occupancy values outside of [0;100] will throw std::invalid_argument.
		 */
		void setOccupancy(const grid_map::Position& pos, const float& value);

		/**
		 * Set the occupancy value at the given index.
		 * Occupancy values outside of [0;100] will throw std::invalid_argument.
		 */
		void setOccupancy(const grid_map::Index& pos, const float& value);

		/**
		 * Conservatively set the occupancy value at the given position.
		 * "Conservatively" means that if the given location is
		 * already marked as occupied, it cannot be marked as free,
		 * and if it is known, it cannot be marked as unknown.
		 */
		void setOccupancyConservative(const grid_map::Position& pos, const float& value);

		/**
		 * Conservatively set the occupancy value at the given index.
		 * "Conservatively" means that if the given location is
		 * already marked as occupied, it cannot be marked as free,
		 * and if it is known, it cannot be marked as unknown.
		 */
		void setOccupancyConservative(const grid_map::Index& pos, const float& value);
		
		/**
		 * Set the occupancy values at multiple positions.
		 * Occupancy values outside of [0;100] will throw std::invalid_argument.
		 * @param x List of x positions at which to set the values.
		 * @param y List of y positions at which to set the values.
		 * @param values List of occupancy values to set at the corresponding positions.
		 * @param n Number of values to set.
		 */
		void setOccupancy(
				const std::vector<double>& x,
				const std::vector<double>& y,
				const std::vector<float>& values,
				const unsigned int n);

		/**
		 * Conservatively set multiple values.
		 */
		void setOccupancyConservative(
				const std::vector<double>& x,
				const std::vector<double>& y,
				const std::vector<float>& values,
				const unsigned int n);

		/**
		 * Conservatively merge another map with this one.
		 */
		void merge(const RobotMap& other);
	};

};
