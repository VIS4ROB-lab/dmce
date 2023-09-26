#include "mre_dmcts_core/FrontierClustering.hpp"

namespace mre_dmcts {
	FrontierClustering::FrontierClustering()
	{
		reset();
	}

	void FrontierClustering::reset() {
		resolution_ = 0;
		mapOffset_ = {0, 0};
		clusterSizes_.clear();
		clusterCentroids_.clear();
		frontierCells_.clear();
	}

	void FrontierClustering::fromMapFrontiers(const OccupancyMap& map) {
		reset();
		resolution_ = map.getResolution();
		map.getPosition({0,0}, mapOffset_);
		for (auto it = map.getGridMapIterator(); !it.isPastEnd(); ++it) {
			if (map.isFrontierCell(*it))
				addFrontierCell_(*it, map);
		}
		calculateClusterCentroids_();
	}

	void FrontierClustering::addFrontierCell_(const index_t& cellIdx, const OccupancyMap& map) {
		if (alreadyFound(cellIdx))
			return;

		std::vector<index_t> directionsToCheck{{0, -1}, {0, 1}, {1, 0}, {-1, 0}};
		std::vector<unsigned int> neighboringClusters;
		for (const index_t& direction : directionsToCheck) {
			index_t neighbor = cellIdx + direction;
			if (alreadyFound(neighbor))
				neighboringClusters.push_back(frontierCells_[neighbor]);
		}

		if (neighboringClusters.size() > 0) {
			unsigned int mergedClusterId = mergeClusters_(neighboringClusters);
			frontierCells_[cellIdx] = mergedClusterId;
			clusterSizes_[mergedClusterId] += 1;
			return;
		}

		frontierCells_[cellIdx] = clusterCentroids_.size();
		clusterCentroids_.push_back({0,0});
		clusterSizes_.push_back(1);
	}

	unsigned int FrontierClustering::mergeClusters_(
			const std::vector<unsigned int>& clustersToMerge
	) {
		if (clustersToMerge.empty())
			throw std::invalid_argument("Input vector cannot be empty!");
		if (clustersToMerge.size() == 1)
			return clustersToMerge[0];

		unsigned int mergedClusterId = clustersToMerge[0];
		for (unsigned int i = 1; i < clustersToMerge.size(); ++i)
			mergedClusterId = mergeClusters_(mergedClusterId, clustersToMerge[i]);

		eraseEmptyClusters_();
		return mergedClusterId;
	}

	unsigned int FrontierClustering::mergeClusters_(unsigned int a, unsigned int b) {
		if (a == b)
			return a;
		unsigned int mergedClusterId = std::min(a, b);
		unsigned int otherId = std::max(a, b);
		if (otherId >= getNClusters())
			throw std::out_of_range("Cluster ID out of range!");

		for (const auto& element : frontierCells_) {
			index_t cellIdx = element.first;
			unsigned int clusterIdx = element.second;
			if (clusterIdx == otherId)
				frontierCells_[cellIdx] = mergedClusterId;
		}
		clusterSizes_[mergedClusterId] += clusterSizes_[otherId];
		clusterSizes_[otherId] = 0;
		return mergedClusterId;
	}

	void FrontierClustering::eraseEmptyCluster_(unsigned int clusterId) {
		if (clusterId >= getNClusters())
			throw std::out_of_range("Cluster ID out of range!");
		if (clusterSizes_[clusterId] > 0)
			throw std::invalid_argument("Cluster is not empty!");

		for (const auto& element : frontierCells_) {
			index_t cellIdx = element.first;
			unsigned int cellCluster = element.second;
			if (cellCluster > clusterId)
				frontierCells_[cellIdx] -= 1;
		}
		clusterSizes_.erase(clusterSizes_.begin() + clusterId);
		clusterCentroids_.erase(clusterCentroids_.begin() + clusterId);
	}

	void FrontierClustering::eraseEmptyClusters_() {
		for (unsigned int i = 0; i < getNClusters(); ++i)
			if (clusterSizes_[i] == 0)
				eraseEmptyCluster_(i);
	}

	void FrontierClustering::calculateClusterCentroids_() {
		std::fill(clusterCentroids_.begin(), clusterCentroids_.end(), pos_t{0, 0});
		for (const auto& element : frontierCells_) {
			index_t cellIdx = element.first;
			unsigned int clusterIdx = element.second;
			clusterCentroids_[clusterIdx] += indexToPosition(cellIdx);
		}
		for (unsigned int i = 0; i < clusterSizes_.size(); i++)
			clusterCentroids_[i] /= clusterSizes_[i];
	}

	bool FrontierClustering::alreadyFound(const index_t& idx) const {
		return frontierCells_.count(idx) > 0;
	}

	unsigned int FrontierClustering::getNClusters() const {
		return clusterCentroids_.size();
	}

	std::vector<pos_t> FrontierClustering::getCentroidPositions() const {
		return clusterCentroids_;
	}

	std::vector<unsigned int> FrontierClustering::getClusterSizes() const {
		return clusterSizes_;
	}

	pos_t FrontierClustering::indexToPosition(const index_t& index) const {
		if (index.x() < 0 || index.y() < 0)
			throw std::range_error("Index cannot be negative!");
		return mapOffset_ - pos_t{resolution_*index.x(), resolution_*index.y()};
	}

	mre_dmcts_msgs::FrontierClusters FrontierClustering::getMessage() const {
		mre_dmcts_msgs::FrontierClusters msg;
		msg.nClusters = getNClusters();
		msg.mapResolution = resolution_;
		msg.mapOffset_x = mapOffset_.x();
		msg.mapOffset_y = mapOffset_.y();
		for (unsigned int i = 0; i < getNClusters(); ++i) {
			msg.clusterCentroids_x.push_back(clusterCentroids_[i].x());
			msg.clusterCentroids_y.push_back(clusterCentroids_[i].y());
			msg.clusterSizes.push_back(clusterSizes_[i]);
		}
		for (const auto& element : frontierCells_) {
			msg.cellIndices_x.push_back(element.first.x());
			msg.cellIndices_y.push_back(element.first.y());
			msg.cellClusterAssignment.push_back(element.second);
		}
		return msg;
	}

	void FrontierClustering::fromMessage(const mre_dmcts_msgs::FrontierClusters& msg) {
		reset();
		resolution_ = msg.mapResolution;
		mapOffset_ = pos_t{msg.mapOffset_x, msg.mapOffset_y};
		for (unsigned int i = 0; i < msg.nClusters; ++i) {
			clusterCentroids_.push_back({msg.clusterCentroids_x[i], msg.clusterCentroids_y[i]});
			clusterSizes_.push_back(msg.clusterSizes[i]);
		}
		for (unsigned int i = 0; i < msg.cellIndices_x.size(); ++i) {
			index_t cellIdx{msg.cellIndices_x[i], msg.cellIndices_y[i]};
			frontierCells_[cellIdx] = msg.cellClusterAssignment[i];
		}
	}

	indexList_t FrontierClustering::getCellsByCluster(unsigned int clusterId) const {
		if (clusterId >= getNClusters())
			throw std::out_of_range("Cluster ID out of range!");
		indexList_t result;
		for (const auto& element : frontierCells_)
			if (element.second == clusterId)
				result.push_back(element.first);
		return result;
	}

	std::vector<std::pair<unsigned int, double>> FrontierClustering::sortClustersByDistance(
			const pos_t& referencePosition) const
	{
		using pair_t = std::pair<unsigned int, double>;
		std::vector<pair_t> result;
		for (unsigned int i = 0; i < getNClusters(); i++) {
			double squaredDistance = (clusterCentroids_[i] - referencePosition).squaredNorm();
			result.push_back({i, squaredDistance});
		}

		auto ordering = [](const pair_t& l, const pair_t& r) -> bool {
			return l.second < r.second;
		};

		std::sort(result.begin(), result.end(), ordering);

		return result;
	}
}

