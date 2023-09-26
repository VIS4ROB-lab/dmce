#pragma once

#include <random>
#include <stdexcept>
#include <vector>

#include "utils.hpp"

namespace dmce {
	/**
	 * Class representing a log with a finite number of
	 * records. When the log is full and records are added,
	 * the oldest record is overridden.
	 */
	template<typename EntryType>
	class FiniteLog {
		size_t size_ = 0;
		size_t bufferSize_;
		std::vector<EntryType> logData_;
		size_t oldestIdx_;
		bool empty_ = true;
	
	public:
		using iterator = typename std::vector<EntryType>::iterator;

		FiniteLog(size_t logSize)
			: bufferSize_(logSize), oldestIdx_(0)
		{
			logData_.reserve(bufferSize_);
		}

		~FiniteLog() {

		}

		void push(EntryType entry) {
			if (size_ < bufferSize_) {
				++size_;
				logData_.push_back(entry);
			} else {
				logData_[oldestIdx_] = entry;
			}

			oldestIdx_ = (oldestIdx_+1) % bufferSize_;
			empty_ = false;
		}

		template<typename R>
		void reduce(R& result, void(*func)(R&, const EntryType&)) const {
			for (size_t i = 0; i < size_; i++) {
				func(result, logData_[(oldestIdx_+i)%bufferSize_]);
			}
		}

		const EntryType& randomItem() const {
			if (empty())
				throw std::runtime_error("[FiniteLog::randomItem] called on empty log!");

			size_t idx = utils::randomIndex(size());
			return logData_[(oldestIdx_-1-idx)%size()];
		}

		size_t size() const {
			return size_;
		}

		bool empty() const {
			return empty_;
		}

		iterator begin() {
			return logData_.begin();
		}

		iterator begin() const {
			return logData_.begin();
		}

		iterator end() {
			return logData_.end();
		}

		iterator end() const {
			return logData_.end();
		}
	};
}
