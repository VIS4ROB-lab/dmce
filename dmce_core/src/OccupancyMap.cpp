#include "dmce_core/OccupancyMap.hpp"

namespace dmce {
	OccupancyMap::OccupancyMap() : OccupancyMap({1, 1}, 0.05, {0, 0}, "map") {};

	OccupancyMap::OccupancyMap(const grid_map::Length& l,
			 const double& r,
			 const grid_map::Position& p,
			 const std::string& id,
			 const double& initialValue)
		: map_({"occupancy"}), nUnknownCells_(0)
	{
		map_.setGeometry(l, r, p);
		map_.setFrameId(id);
		for (grid_map::GridMapIterator it(map_); !it.isPastEnd(); ++it) {
			map_.at("occupancy", *it) = initialValue;
		}

		if (!isKnown(initialValue)) {
			nUnknownCells_ = getNCells();
		}
	}

	OccupancyMap::OccupancyMap(const grid_map_msgs::GridMap& message) {
		grid_map::GridMapRosConverter::fromMessage(message, map_, {"occupancy"});
		countUnknownCells();
	}

	float OccupancyMap::getOccupancy(const grid_map::Position& pos) const {
		return map_.atPosition("occupancy", pos);
	}

	float OccupancyMap::getOccupancy(const grid_map::Index& idx) const {
		return map_.at("occupancy", idx);
	}

	grid_map::GridMapIterator OccupancyMap::getGridMapIterator() const {
		return grid_map::GridMapIterator(map_);
	}

	grid_map::CircleIterator OccupancyMap::getCircleIterator(
			const grid_map::Position& center,
			const double radius) const
	{
		return grid_map::CircleIterator(map_, center, radius);
	}

	grid_map::LineIterator OccupancyMap::getLineIterator(
			const grid_map::Position& from,
			const grid_map::Position& to) const
	{
		return grid_map::LineIterator(map_, from, to);
	}

	grid_map::SubmapIterator OccupancyMap::getSubmapIterator(
			const pos_t& position,
			const pos_t& size) const
	{
		bool success;
		grid_map::SubmapGeometry geom(map_, position, size, success);
		if (!success) {
			ROS_ERROR("[OccupancyMap::getSubmapIterator] Failed to initialise submap geometry!");
		}
		return grid_map::SubmapIterator(geom);
	}
	
	/**
	 * Return a GridMap message representing this map.
	 */
	grid_map_msgs::GridMap OccupancyMap::getMessage() const {
		grid_map_msgs::GridMap message;
		grid_map::GridMapRosConverter::toMessage(map_, message);
		return message;
	}


	grid_map_msgs::GridMap OccupancyMap::getSubmapMessage(
			const pos_t& position,
			const grid_map::Length& size,
			bool& success) const
	{
		grid_map::GridMap subMap = map_.getSubmap(position, size, success);
		grid_map_msgs::GridMap message;
		grid_map::GridMapRosConverter::toMessage(subMap, message);
		return message;
	}

	bool OccupancyMap::getPosition(
			const grid_map::Index& index,
			grid_map::Position& position) const
	{
		return map_.getPosition(index, position);
	}

	const grid_map::Position& OccupancyMap::getPosition() const {
		return map_.getPosition();
	}

	const grid_map::Length& OccupancyMap::getLength() const {
		return map_.getLength();
	}

	const grid_map::Index& OccupancyMap::getSize() const {
		return map_.getSize();
	}

	double OccupancyMap::getResolution() const {
		return map_.getResolution();
	}
	
	const std::string& OccupancyMap::getFrameId() const {
		return map_.getFrameId();
	}

	bool OccupancyMap::isOccupied(const grid_map::Index& idx) const {
		return getOccupancy(idx) > occupiedThreshold_;
	}

	bool OccupancyMap::isFree(const grid_map::Index& idx) const {
		return getOccupancy(idx) < freeThreshold_;
	}

	bool OccupancyMap::isKnown(const grid_map::Index& idx) const {
		return isFree(idx) || isOccupied(idx);
	}

	bool OccupancyMap::isOccupied(const grid_map::Position& pos) const {
		return getOccupancy(pos) > occupiedThreshold_;
	}

	bool OccupancyMap::isFree(const grid_map::Position& pos) const {
		return getOccupancy(pos) < freeThreshold_;
	}

	bool OccupancyMap::isKnown(const grid_map::Position& pos) const {
		return isFree(pos) || isOccupied(pos);
	}

	bool OccupancyMap::isOccupied(const double& value) const {
		return value > occupiedThreshold_;
	}

	bool OccupancyMap::isFree(const double& value) const {
		return value < freeThreshold_;
	}

	bool OccupancyMap::isKnown(const double& value) const {
		return isFree(value) || isOccupied(value);
	}

	using Costmap2DTranslationTable = grid_map::Costmap2DTranslationTable<50, 100, 99, 80>;

	costmap_2d::Costmap2D OccupancyMap::toCostmap() const {
		costmap_2d::Costmap2D cmap;
		grid_map::Costmap2DConverter<grid_map::GridMap, Costmap2DTranslationTable> converter;
		converter.initializeFromGridMap(map_, cmap);
		bool success = converter.setCostmap2DFromGridMap(map_, "occupancy", cmap);
		return cmap;
	}

	cv::Mat OccupancyMap::toImage() const {
		cv::Mat image_msg;
		std::string layer = "occupancy";
		const float minValue = freeValue;
		const float maxValue = occupiedValue;
		const bool hasNaN = map_.get(layer).hasNaN();
		bool success = true;
		if (hasNaN) {
			success = grid_map::GridMapCvConverter::toImage<unsigned char, 4>(
				map_, layer, CV_8UC4, minValue, maxValue, image_msg
			);
		} else {
			success = grid_map::GridMapCvConverter::toImage<unsigned char, 1>(
				map_, layer, CV_8UC1, minValue, maxValue, image_msg
			);
		}

		if (!success) {
			throw std::runtime_error(
				"[OccupancyMap::toImage] Failed to convert grid_map "
				"to OpenCV image!"
			);
		}

		return image_msg;
	}


	void OccupancyMap::fromImage(
			const cv::Mat& image,
			const double& resolution,
			const grid_map::Position pos
	) {
		auto frameId = map_.getFrameId();
		map_.setFrameId(frameId);
		sensor_msgs::Image image_msg = *(
			cv_bridge::CvImage(std_msgs::Header(), "mono8", image).toImageMsg()
		);
		grid_map::GridMapRosConverter::initializeFromImage(
			image_msg, resolution, map_
		);
		grid_map::GridMapRosConverter::addLayerFromImage(
			image_msg, "occupancy", map_, 0.0, 100.0
		);
		map_.setPosition(pos);
		countUnknownCells();
	}

	cv::Mat OccupancyMap::inflateObstacles(
		const cv::Mat& sourceImage,
		unsigned int amount)
	{
		cv::Mat destImage;
		cv::Mat kernel = getStructuringElement(
			cv::MORPH_ELLIPSE,
			cv::Size(2*amount + 1, 2*amount+1),
			cv::Point(amount, amount)
		);
		cv::dilate(sourceImage, destImage, kernel);
		return destImage;
	}

	bool OccupancyMap::isFrontierCell(const grid_map::Index& idx) const {
		if (!isIndexInRange(idx) || !isFree(idx))
			return false;

		auto checkDirection = [&](grid_map::Index idx, grid_map::Index offset) -> bool {
			grid_map::Index toCheck = idx + offset;
			if (isIndexInRange(toCheck) && !isKnown(toCheck)) {
				return true;
			}
			return false;
		};

		return (checkDirection(idx, {-1, 0})
				|| checkDirection(idx, { 1, 0})
				|| checkDirection(idx, { 0,-1})
				|| checkDirection(idx, { 0,1})
				|| checkDirection(idx, { 1, 1})
				|| checkDirection(idx, { 1,-1})
				|| checkDirection(idx, {-1, 1})
				|| checkDirection(idx, {-1,-1}));
	}

	bool OccupancyMap::isFrontierCell(const pos_t& pos) const {
		grid_map::Index idx;
		return map_.getIndex(pos, idx) && isFrontierCell(idx);
	}

	indexList_t OccupancyMap::getFrontier() const {
		indexList_t frontier;
		for (auto it = getGridMapIterator(); !it.isPastEnd(); ++it) {
			auto idx = *it;
			if (isFrontierCell(idx))
				frontier.push_back(idx);
		}
		return frontier;
	}

	bool OccupancyMap::isIndexInRange(const grid_map::Index& idx) const {
		bool isOutside = false;
		isOutside |= idx.x() < 0;
		isOutside |= idx.y() < 0;
		auto size = map_.getSize();
		isOutside |= idx.x() >= size.x();
		isOutside |= idx.y() >= size.y();
		return !isOutside;
	}

	bool OccupancyMap::isInside(const grid_map::Position& pos) const {
		return map_.isInside(pos);
	}

	double OccupancyMap::getRelativeEntropy() const {
		return getNUnknownCells() / (double) getNCells();
	}

	unsigned int OccupancyMap::getNCells() const {
		auto size = map_.getSize();
		return size.x() * size.y();
	}

	unsigned int OccupancyMap::getNUnknownCells() const {
		return nUnknownCells_;
	}

	void OccupancyMap::countUnknownCells() {
		nUnknownCells_ = 0;
		auto it = getGridMapIterator();
		for ( ; !it.isPastEnd(); ++it) {
			if (!isKnown(*it))
				nUnknownCells_++;
		}
	}

	void OccupancyMap::inflateObstacles(double radius) {
		fromImage(inflateObstacles(toImage(), radius / getResolution()), getResolution());
	}
};

