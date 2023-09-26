#pragma once

#include "dmce_core/OccupancyMap.hpp"
#include "dmce_msgs/FrontierClusters.h"

namespace dmce {
	/**
	 * This map is used to identify frontier clusters in an OccupancyMap.
	 * Use .fromMapFrontiers to read a map and generate frontier clusters.
	 * Use .fromMessage to read a ROS message previously generated with .getMessage.
	 */
	class FrontierClustering {
		std::vector<pos_t> clusterCentroids_;
		std::vector<unsigned int> clusterSizes_;
		std::map<index_t, unsigned int> frontierCells_;
		double resolution_;
		pos_t mapOffset_;

	public:
		/**
		 * Empty initialisation.
		 */
		FrontierClustering();

		/**
		 * Clear all saved cells and clusters.
		 */
		void reset();

		/**
		 * Compute clusters of frontier cells found in given map.
		 */
		void fromMapFrontiers(const OccupancyMap& map);

		/**
		 * Returns the number of clusters.
		 */
		unsigned int getNClusters() const;

		/**
		 * Returns the centroid position of each cluster.
		 */
		std::vector<pos_t> getCentroidPositions() const;

		/**
		 * Returns the number of cells assigned to each cluster.
		 */
		std::vector<unsigned int> getClusterSizes() const;

		/**
		 * Returns position corresponding to index in the global frame of reference
		 * (i.e. accounting for map position)
		 * Note: performs no bounds checking!
		 */
		pos_t indexToPosition(const index_t& index) const;

		/**
		 * Returns a FrontierClusters message representing the current state.
		 */
		dmce_msgs::FrontierClusters getMessage() const;

		/**
		 * Reads contents of a FrontierClusters message.
		 */
		void fromMessage(const dmce_msgs::FrontierClusters& message);

		/**
		 * Returns a vector of indices of the cells assigned to the given cluster ID.
		 */
		indexList_t getCellsByCluster(unsigned int clusterId) const;

		/**
		 * Sort the clusters based on their centroid's distance to the given point (ascending).
		 * Returns a sorted vector of pairs: pair.first is the centroid ID, pair.second is
		 * the squared distance to the reference point.
		 */
		std::vector<std::pair<unsigned int, double>>
			sortClustersByDistance(const pos_t& referencePosition) const;

	private:
		/**
		 * True if the given cell index has already been assigned to a cluster.
		 */
		bool alreadyFound(const index_t& idx) const;

		/**
		 * Add the given frontier cell to the clusters.
		 * This may cause a new cluster to be created, or multiple
		 * clusters to be merged into one.
		 * Note that the cluster centroids are not updated by this operation.
		 */
		void addFrontierCell_(const index_t& idx, const OccupancyMap& map);

		/**
		 * Calculate the centroid of each cluster as the mean of the
		 * positions of cells assigned to it.
		 */
		void calculateClusterCentroids_();

		/**
		 * Merge two or more clusters. Clusters emptied this way will be erased.
		 * Note that the cluster centroids are not updated by this operation.
		 */
		unsigned int mergeClusters_(const std::vector<unsigned int>& clustersToMerge);

		/**
		 * Merge two clusters.
		 * Cells assigned to cluster max(a,b) will be reassigned to min(a,b).
		 * Cluster max(a,b) will be left empty, but not erased.
		 * Note that the cluster centroids are not updated by this operation.
		 */
		unsigned int mergeClusters_(unsigned int a, unsigned int b);

		/**
		 * Remove the given cluster, updating the IDs of clusters with larger IDs.
		 * Throws an error if cluster is not empty or out of range.
		 */
		void eraseEmptyCluster_(unsigned int clusterId);

		/**
		 * Erase each empty cluster.
		 */
		void eraseEmptyClusters_();
	};
}
