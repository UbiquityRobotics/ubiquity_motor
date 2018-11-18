/**
Copyright (c) 2016, Ubiquity Robotics
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of ubiquity_motor nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
**/

#ifndef SHAREDQUEUE_H
#define SHAREDQUEUE_H

#include <atomic>
#include <mutex>
#include <queue>

template <typename T>
class shared_queue {
public:
    shared_queue() : queue_empty_(true){};
    ~shared_queue(){};

    shared_queue(const shared_queue& other) {
        std::lock_guard<std::mutex> other_lock(other.queue_mutex_);
        std::lock_guard<std::mutex> this_lock(queue_mutex_);

        // Copy is not atomic
        bool qe = other.queue_empty_;
        queue_empty_ = qe;

        internal_queue_ = other.internal_queue_;
    };

    shared_queue& operator=(const shared_queue& other) {
        std::lock_guard<std::mutex> other_lock(other.queue_mutex_);
        std::lock_guard<std::mutex> this_lock(queue_mutex_);

        // Copy is not atomic
        bool qe = other.queue_empty_;
        queue_empty_ = qe;

        internal_queue_ = other.internal_queue_;

        return *this;
    };

    void push(const T& value) {
        std::lock_guard<std::mutex> lock(queue_mutex_);
        internal_queue_.push(value);

        queue_empty_ = internal_queue_.empty();
    };

    void push(const std::vector<T>& values) {
        std::lock_guard<std::mutex> lock(queue_mutex_);

        for (typename std::vector<T>::const_iterator it = values.begin();
             it != values.end(); ++it) {
            internal_queue_.push(*it);
            queue_empty_ = internal_queue_.empty();
        }
    };

    T& front() {
        std::lock_guard<std::mutex> lock(queue_mutex_);
        return internal_queue_.front();
    };

    const T& front() const {
        std::lock_guard<std::mutex> lock(queue_mutex_);
        return internal_queue_.front();
    };

    T front_pop() {
        std::lock_guard<std::mutex> lock(queue_mutex_);

        T value = internal_queue_.front();
        internal_queue_.pop();
        queue_empty_ = internal_queue_.empty();

        return value;
    };

    void pop() {
        std::lock_guard<std::mutex> lock(queue_mutex_);
        internal_queue_.pop();

        queue_empty_ = internal_queue_.empty();
    };

    bool empty() const {
        std::lock_guard<std::mutex> lock(queue_mutex_);
        return internal_queue_.empty();
    }

    bool fast_empty() const { return queue_empty_; }

    size_t size() const {
        std::lock_guard<std::mutex> lock(queue_mutex_);
        return internal_queue_.size();
    }

private:
    mutable std::mutex queue_mutex_;
    std::atomic<bool> queue_empty_;
    std::queue<T> internal_queue_;
};

#endif