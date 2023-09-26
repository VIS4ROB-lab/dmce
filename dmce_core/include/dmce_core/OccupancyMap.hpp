#pragma once

#include <ros/ros.h>
#include "costmap_2d/costmap_2d.h"
#include "grid_map_costmap_2d/grid_map_costmap_2d.hpp"
#include "grid_map_core/GridMap.hpp"
#include "grid_map_msgs/GridMap.h"
#include "grid_map_ros/grid_map_ros.hpp"
#include "grid_map_cv/GridMapCvConverter.hpp"

#include "dmce_core/TypeDefs.hpp"

namespace dmce {

	/**
	 * A 2D occupancy grid.
	 * This class does not implement any setters and is therefore read-only.
	 */
	class OccupancyMap {
	protected:
		constexpr static const float occupiedThreshold_ = 80;
		constexpr static const float freeThreshold_ = 20;

		grid_map::GridMap map_;
		unsigned int nUnknownCells_;

	public:
		constexpr static const float freeValue = 0;
		constexpr static const float occupiedValue= 100;
		constexpr static const float unknownValue = 50;

		/**
		 * Default constructor.
		 */
		OccupancyMap();

		/**
		 * Constructor. Arguments correspond to the arguments of
		 *  grid_map::GridMap::setGeometry and ::setFrameId.
		 * @param l [m] Dimensions of the map.
		 * @param r [m/cell] Resolution of the map.
		 * @param p [m] Position of the map.
		 * @param id Frame ID of the map.
		 * @param initialValue Initial occupancy value with which the map is filled.
		 */
		OccupancyMap(const grid_map::Length& length,
				 const double& resolution,
				 const grid_map::Position& position,
				 const std::string& frameId,
				 const double& initialValue = 50);

		/**
		 * Construct from a GridMap message.
		 */
		OccupancyMap(const grid_map_msgs::GridMap& message);


		OccupancyMap(const OccupancyMap&) = default;
		OccupancyMap& operator=(const OccupancyMap&) = default;
		OccupancyMap(OccupancyMap&&) = default;
		OccupancyMap& operator=(OccupancyMap&&) = default;

		/**
		 * Return the occupancy value at the given position.
		 */
		float getOccupancy(const grid_map::Position& pos) const;

		/**
		 * Return the occupancy value at the given index.
		 */
		float getOccupancy(const grid_map::Index& idx) const;

		/**
		 * Returns true if cell at given index is occupied.
		 */
		bool isOccupied(const grid_map::Index& idx) const;

		/**
		 * Returns true if cell at given index is known to be free (no obstacles).
		 */
		bool isFree(const grid_map::Index& idx) const;

		/**
		 * Returns true if cell at given index is known (not uncertain).
		 */
		bool isKnown(const grid_map::Index& idx) const;

		/**
		 * Returns true if cell at given position is occupied.
		 */
		bool isOccupied(const grid_map::Position& idx) const;

		/**
		 * Returns true if cell at given position is known to be free (no obstacles).
		 */
		bool isFree(const grid_map::Position& idx) const;

		/**
		 * Returns true if cell at given position is known (not uncertain).
		 */
		bool isKnown(const grid_map::Position& idx) const;

		/**
		 * Returns true if given value counts as occupied.
		 */
		bool isOccupied(const double& value) const;

		/**
		 * Returns true if given value counts as free.
		 */
		bool isFree(const double& value) const;

		/**
		 * Returns true if given value counts as known (free or occupied).
		 */
		bool isKnown(const double& value) const;

		/**
		 * Return a GridMapIterator over the whole index space.
		 */
		grid_map::GridMapIterator getGridMapIterator() const;

		/**
		 * Return a CircleIterator over the given circular area.
		 */
		grid_map::CircleIterator getCircleIterator(
				const grid_map::Position& center,
				const double radius) const;

		/**
		 * Return LineIterator between points from and to.
		 */
		grid_map::LineIterator getLineIterator(
				const grid_map::Position& from,
				const grid_map::Position& to) const;

		/**
		 * Return SubMapIterator centered on position and
		 * with given dimensions.
		 */
		grid_map::SubmapIterator getSubmapIterator(
				const pos_t& position,
				const pos_t& size) const;
		
		/**
		 * Return a GridMap message representing this map.
		 */
		grid_map_msgs::GridMap getMessage() const;

		/**
		 * Return a GridMap message representing the given submap.
		 */
		grid_map_msgs::GridMap getSubmapMessage(
				const pos_t& position,
				const grid_map::Length& size,
				bool& success) const;

		/*!
		 * Gets the 2d position of cell specified by the index (x, y of cell position)
		 * in the grid map frame.
		 * @param[in] index the index of the requested cell.
		 * @param[out] position the position of the data point in the parent frame.
		 * @return true if successful, false if index not within range of buffer.
		 */
		bool getPosition(
				const grid_map::Index& index,
				grid_map::Position& position) const;

		/**
		 * Convert to a costmap_2d object.
		 * Passed to OMPL for path planning.
		 */
		costmap_2d::Costmap2D toCostmap() const;

		/**
		 * Convert occupancy data to an OpenCV image object.
		 */
		cv::Mat toImage() const;

		/**
		 * Overwrite occupancy data with data from OpenCV image.
		 */
		void fromImage(
			const cv::Mat& sourceImage,
			const double& resolution,
			const grid_map::Position pos = {0, 0}
		);

		/**
		 * Inflate the obstacles in the image by applying a dilation filter.
		 * @param sourceImage Initial image.
		 * @param amount Radius of the inflation, in pixels.
		 */
		static cv::Mat inflateObstacles(
			const cv::Mat& sourceImage,
			unsigned int amount
		);

		/**
		 * Inflate obstacles in this map by the given radius.
		 */
		void inflateObstacles(double radius);

		bool isFrontierCell(const grid_map::Index& idx) const;

		bool isFrontierCell(const pos_t& pos) const;

		indexList_t getFrontier() const;

		const grid_map::Position& getPosition() const;

		const grid_map::Length& getLength() const;

		const grid_map::Index& getSize() const;

		double getResolution() const;
		
		const std::string& getFrameId() const;

		unsigned int getNCells() const;

		unsigned int getNUnknownCells() const;

		/**
		 * Returns true if given index is within the map bounds.
		 */
		bool isIndexInRange(const grid_map::Index& idx) const;

		/**
		 * Returns true if the given position is within map bounds.
		 */
		bool isInside(const grid_map::Position& pos) const;

		/**
		 * Returns a value in range [0,1] representing the relative
		 * entropy (% of cells known) of the map.
		 */
		double getRelativeEntropy() const;

		/**
		 * Recounts how many undiscovered cells there are.
		 */
		void countUnknownCells();
	};
};

