#ifndef PAIRWISE_ITERATOR_HPP
#define PAIRWISE_ITERATOR_HPP

#include <algorithm>
#include <iostream>
#include <boost/iterator/iterator_facade.hpp>

template <typename iterable>
class pairwise_iterator
  : public boost::iterator_facade<
        pairwise_iterator<iterable>
      , typename iterable::value_type
      , boost::forward_traversal_tag
      , std::pair<typename iterable::value_type&, typename iterable::value_type&>
      , typename iterable::difference_type
    >
{
public:
    typedef typename iterable::difference_type difference_type;
    typedef typename iterable::iterator iterator;
    typedef typename iterable::value_type Value;

    pairwise_iterator()

    {
    }

    explicit pairwise_iterator(iterator i)
        : _i(i), _j(i)
    {
        ++_j;
    }

private:
    friend class boost::iterator_core_access;

    bool equal(pairwise_iterator<iterable> const& other) const
    {
        return _j == other._i;
    }

    void increment()
    {
        ++_i;
        ++_j;
    }

    std::pair<Value&, Value&> dereference() const
    {
        return std::make_pair(std::ref(*_i), std::ref(*(_j)));
    }

    void advance(difference_type n)
    {
        std::advance(_i, n);
    }

    difference_type distance_to(pairwise_iterator<Value> const& other) const
    {
        return std::distance(_i, other._i);
    }

    iterator _i, _j;
};

template <typename iterable>
class pairwise_const_iterator
  : public boost::iterator_facade<
        pairwise_const_iterator<iterable>
      , typename iterable::value_type const
      , boost::forward_traversal_tag
      , std::pair<typename iterable::value_type&, typename iterable::value_type&>
      , typename iterable::difference_type
    >
{
public:
    typedef typename iterable::difference_type difference_type;
    typedef typename iterable::const_iterator const_iterator;
    typedef typename iterable::value_type Value;

    pairwise_const_iterator()

    {
    }

    explicit pairwise_const_iterator(const_iterator i)
        : _i(i), _j(i)
    {
        ++_j;
    }

private:
    friend class boost::iterator_core_access;

    bool equal(pairwise_const_iterator<iterable> const& other) const
    {
        return _j == other._i;
    }

    void increment()
    {
        ++_i;
        ++_j;
    }

    std::pair<const Value&,const Value&> dereference() const
    {
        return std::make_pair(std::ref(*_i), std::ref(*(_j)));
    }

    void advance(difference_type n)
    {
        std::advance(_i, n);
    }

    difference_type distance_to(pairwise_iterator<Value> const& other) const
    {
        return std::distance(_i, other._i);
    }

    const_iterator _i, _j;
};

#endif // PAIRWISE_ITERATOR_HPP
