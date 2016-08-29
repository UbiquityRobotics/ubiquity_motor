#include <gtest/gtest.h>

#include <ubiquity_motor/shared_queue.h>

TEST(SharedQueueTest, emptyOnConstuct) {
	shared_queue<int> sq;

	ASSERT_TRUE(sq.fast_empty());
	ASSERT_TRUE(sq.empty());
}

TEST(SharedQueueTest, notEmptyAfterPush) {
	shared_queue<int> sq;
	sq.push(25);

	ASSERT_FALSE(sq.fast_empty());
	ASSERT_FALSE(sq.empty());
}

TEST(SharedQueueTest, emptyAfterPushAndPop) {
	shared_queue<int> sq;
	sq.push(25);

	ASSERT_FALSE(sq.fast_empty());
	ASSERT_FALSE(sq.empty());

	sq.pop();

	ASSERT_TRUE(sq.fast_empty());
	ASSERT_TRUE(sq.empty());
}

TEST(SharedQueueTest, pushFrontPop) {
	shared_queue<int> sq;

	for (int i = 0; i < 5; ++i){
		sq.push(i);
	}

	for (int i = 0; i < 5; ++i){
		ASSERT_EQ(i, sq.front());
		sq.pop();
	}
}

TEST(SharedQueueTest, pushPopSize) {
	shared_queue<int> sq;

	for (int i = 0; i < 5; ++i){
		sq.push(i);
		ASSERT_EQ(i, sq.size());
	}

	ASSERT_EQ(5, sq.size());

	for (int i = 0; i < 5; ++i){
		ASSERT_EQ(5-i, sq.size());
		sq.pop();
	}
}

TEST(SharedQueueTest, pushSize) {
	shared_queue<int> sq;

	for (int i = 0; i < 5; ++i){
		sq.push(i);
	}

	ASSERT_EQ(5, sq.size());
}

int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}