// File:          concurrent_queue.hpp
// Creation Date: 11/16/2012
// all rights reserved

// derived from work by Anthony Williams http://www.justsoftwaresolutions.co.uk/threading/implementing-a-thread-safe-queue-using-condition-variables.html


#ifndef CONCURRENT_QUEUE_HPP
#define CONCURRENT_QUEUE_HPP

#include <deque>

#include <boost/thread.hpp>

template<typename Data>
class concurrent_queue
{
private:
    std::deque<Data> the_queue;
    mutable boost::mutex the_mutex;
    boost::condition_variable the_condition_variable;
public:
    void push(Data const& data)
    {
        boost::mutex::scoped_lock lock(the_mutex);
        the_queue.push_back(data);
        lock.unlock();
        the_condition_variable.notify_one();
    }

    bool try_push_maxsize(Data const& data, int max_size)
    {
        boost::mutex::scoped_lock lock(the_mutex);
        if( the_queue.size() > max_size )
          return false;

        the_queue.push_back(data);
        lock.unlock();
        the_condition_variable.notify_one();
        return true;
    }

    bool empty() const
    {
        boost::mutex::scoped_lock lock(the_mutex);
        return the_queue.empty();
    }

    bool try_pop(Data& popped_value)
    {
        boost::mutex::scoped_lock lock(the_mutex);
        if(the_queue.empty())
        {
            return false;
        }

        popped_value=the_queue.front();
        the_queue.pop_front();
        return true;
    }

    void wait_and_pop(Data& popped_value)
    {
        boost::mutex::scoped_lock lock(the_mutex);
        while(the_queue.empty())
        {
            the_condition_variable.wait(lock);
        }

        popped_value=the_queue.front();
        the_queue.pop_front();
    }

    // low level access, no notify done!
    void lock() { the_mutex.lock(); }
    void unlock() { the_mutex.unlock(); }
    std::deque<Data>& data() { return the_queue; }
};

#endif // CONCURRENT_QUEUE_HPP
