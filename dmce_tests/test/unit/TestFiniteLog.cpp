#include "dmce_test/common.hpp"

#include "dmce_core/FiniteLog.hpp"

namespace dmce_test {

	using namespace dmce;

	class TestFiniteLog : public testing::Test {
	protected:
		size_t bufSize_ = 10;
		FiniteLog<int> log_;
	public:
		TestFiniteLog() : log_(bufSize_)
		{

		}
	};

	TEST_F(TestFiniteLog, OnInit_isEmpty) {
		EXPECT_TRUE(log_.empty());
	}
	
	TEST_F(TestFiniteLog, onPush_isNotEmpty) {
		log_.push(0);
		EXPECT_FALSE(log_.empty());
		EXPECT_FALSE(log_.begin() == log_.end());
	}

	TEST_F(TestFiniteLog, onPush_sizeIncreasesUpToMax) {
		for (size_t i = 0; i < 2*bufSize_; i++) {
			log_.push(i);
			if (i < bufSize_)
				EXPECT_EQ(log_.size(), i+1);
			else
				EXPECT_EQ(log_.size(), bufSize_);
		}
	}

	TEST_F(TestFiniteLog, onPushOne_firstValueIsCorrect) {
		log_.push(55);
		EXPECT_EQ(*(log_.begin()), 55);
	}

	TEST_F(TestFiniteLog, onPopulate_valuesAreCorrect) {
		for (size_t i = 0; i < bufSize_; i++) {
			log_.push(i*i);
		}

		auto it = log_.begin();
		size_t i = 0;
		for ( ; it < log_.end(); ++it, ++i) {
			EXPECT_EQ(*(it), i*i);
		}

		log_.push(-1);
		EXPECT_EQ(*(log_.begin()), -1);
	}

	TEST_F(TestFiniteLog, onReduce_result_isCorrect) {
		size_t total = 0;
		int reductionResult;
		for (size_t i = 0; i < bufSize_; i++) {
			log_.push(i*i);
			total += i*i;

			reductionResult = 0;
			log_.reduce<int>(reductionResult, [](int& r, const int& el) { r += 1; });
			EXPECT_EQ(reductionResult, i+1);
		}

		reductionResult = 0;
		log_.reduce<int>(reductionResult, [](int& r, const int& el) { r += el; });
		EXPECT_EQ(reductionResult, total);
	}

	TEST_F(TestFiniteLog, onEmpty_randomItem_throwsError) {
		EXPECT_THROW(log_.randomItem(), std::runtime_error);
	}

	TEST_F(TestFiniteLog, onPush_randomSelect_returnsPushedValues) {
		log_.push(42);
		EXPECT_EQ(log_.randomItem(), 42);
		log_.push(73);
		for (size_t i = 0; i < 10; i++) {
			int val = log_.randomItem();
			EXPECT_TRUE(val == 42 || val == 73);
		}
	}
}

