#include <gtest/gtest.h>

#include <ubiquity_motor/shared_queue.h>
#include <boost/thread.hpp>
#include <vector>

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

TEST(SharedQueueTest, notEmptyAfterPushMultiple) {
    shared_queue<int> sq;
    std::vector<int> v;

    for (int i = 0; i < 5; ++i) {
        v.push_back(i);
    }

    sq.push(v);

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

    for (int i = 0; i < 5; ++i) {
        sq.push(i);
    }

    for (int i = 0; i < 5; ++i) {
        ASSERT_EQ(i, sq.front());
        sq.pop();
    }

    for (int i = 0; i < 5; ++i) {
        sq.push(i);
    }

    for (int i = 0; i < 5; ++i) {
        ASSERT_EQ(i, sq.front_pop());
    }

    ASSERT_TRUE(sq.fast_empty());
    ASSERT_TRUE(sq.empty());
    ASSERT_EQ(0, sq.size());
}

TEST(SharedQueueTest, copy) {
    shared_queue<int> sq0;
    shared_queue<int> sq1;

    for (int i = 0; i < 5; ++i) {
        sq0.push(i);
    }

    sq1 = sq0;
    ASSERT_FALSE(sq1.fast_empty());
    ASSERT_FALSE(sq1.empty());
    ASSERT_EQ(5, sq1.size());

    for (int i = 0; i < 5; ++i) {
        ASSERT_EQ(i, sq1.front_pop());
    }

    ASSERT_TRUE(sq1.fast_empty());
    ASSERT_TRUE(sq1.empty());
    ASSERT_EQ(0, sq1.size());

    // Make sure that the original queue is untouched
    ASSERT_FALSE(sq0.fast_empty());
    ASSERT_FALSE(sq0.empty());
    ASSERT_EQ(5, sq0.size());
}

TEST(SharedQueueTest, copyConstruct) {
    shared_queue<int> sq0;

    for (int i = 0; i < 5; ++i) {
        sq0.push(i);
    }

    shared_queue<int> sq1(sq0);
    ASSERT_FALSE(sq1.fast_empty());
    ASSERT_FALSE(sq1.empty());
    ASSERT_EQ(5, sq1.size());

    for (int i = 0; i < 5; ++i) {
        ASSERT_EQ(i, sq1.front_pop());
    }

    ASSERT_TRUE(sq1.fast_empty());
    ASSERT_TRUE(sq1.empty());
    ASSERT_EQ(0, sq1.size());

    // Make sure that the original queue is untouched
    ASSERT_FALSE(sq0.fast_empty());
    ASSERT_FALSE(sq0.empty());
    ASSERT_EQ(5, sq0.size());
}

TEST(SharedQueueTest, pushConstCopyFront) {
    shared_queue<int> sq;

    for (int i = 0; i < 5; ++i) {
        sq.push(i);
    }

    const shared_queue<int> csq = sq;

    ASSERT_FALSE(sq.fast_empty());
    ASSERT_FALSE(sq.empty());
    ASSERT_EQ(5, sq.size());

    const int &front = csq.front();
    ASSERT_EQ(0, front);
}

TEST(SharedQueueTest, pushPopSize) {
    shared_queue<int> sq;

    for (int i = 0; i < 5; ++i) {
        sq.push(i);
        ASSERT_EQ(i + 1, sq.size());
    }

    ASSERT_EQ(5, sq.size());

    for (int i = 0; i < 5; ++i) {
        ASSERT_EQ(5 - i, sq.size());
        sq.pop();
    }
}

TEST(SharedQueueTest, pushMultipleSize) {
    shared_queue<int> sq;
    std::vector<int> v;

    for (int i = 0; i < 5; ++i) {
        v.push_back(i);
    }

    sq.push(v);
    ASSERT_EQ(5, sq.size());
}

void pop_thread(shared_queue<int> *sq) {
    for (int i = 0; i < 100; ++i) {
        ASSERT_FALSE(sq->fast_empty());
        ASSERT_FALSE(sq->empty());
        ASSERT_EQ(i, sq->front());
        sq->pop();
    }
}

TEST(SharedQueueTest, pushFrontPopThreaded) {
    shared_queue<int> sq;

    for (int i = 0; i < 100; ++i) {
        sq.push(i);
    }

    boost::thread popThread = boost::thread(pop_thread, &sq);
    popThread.join();

    ASSERT_TRUE(sq.fast_empty());
    ASSERT_TRUE(sq.empty());
    ASSERT_EQ(0, sq.size());
}

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}