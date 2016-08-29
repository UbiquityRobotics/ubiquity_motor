#ifndef SHAREDQUEUE_H
#define SHAREDQUEUE_H

#include <queue>
#include <boost/thread.hpp>
#include <boost/atomic.hpp>

template <typename T>
class shared_queue
{

public:
	shared_queue() : queue_empty_(true) {};
	~shared_queue() {};

  void push(const T& value) {
    boost::lock_guard<boost::mutex> lock(queue_mutex_); 
    internal_queue_.push(value);

    queue_empty_ = internal_queue_.empty();
  };

  T& front() {
    boost::lock_guard<boost::mutex> lock(queue_mutex_); 
    return internal_queue_.front();
  };

  const T& front() const {
    boost::lock_guard<boost::mutex> lock(queue_mutex_); 
    return internal_queue_.front();
  };

  void pop() {
    boost::lock_guard<boost::mutex> lock(queue_mutex_); 
    internal_queue_.pop();

    queue_empty_ = internal_queue_.empty();
  };

  bool empty() const {
    boost::lock_guard<boost::mutex> lock(queue_mutex_); 
    return internal_queue_.empty();
  }

  bool fast_empty() const {
    return queue_empty_; 
  }

  size_t size() const {
    boost::lock_guard<boost::mutex> lock(queue_mutex_);
    return internal_queue_.size();
  }   
	
private:
  // Disable copy constructors
  shared_queue(const shared_queue& other); // non construction-copyable
  shared_queue& operator=(const shared_queue&); // non copyable

  mutable boost::mutex queue_mutex_;
  boost::atomic<bool> queue_empty_;
  std::queue<T> internal_queue_;
};

#endif