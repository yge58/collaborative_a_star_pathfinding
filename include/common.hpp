#ifndef COMMON_HPP
#define COMMON_HPP


#include <omp.h>
#include <iostream>
#include <cmath>
#include <limits>
#include <utility>
#include <unordered_map>
#include <queue>
#include <list>


using namespace std;
using uint = unsigned int; 
using uchar = unsigned char;

const uint MAX = numeric_limits<uint>::max();
const uint WINDOW_SIZE = 8;



/***************************************************************/
// std priority_queue doesnt have find function. so we create one.
template<class T,
	 class Container = std::vector<T>,
	 class Compare = std::less<typename Container::value_type>>
  class prioriyQueue : public std::priority_queue<T, Container, Compare>
  {
  public:
    typedef typename
    std::priority_queue<T, Container, Compare>::container_type::const_iterator const_iterator;
 
    bool find(const T& val) const
    {
      auto first = this->c.cbegin();
      auto last = this->c.cend();
      while (first != last) {
	if (*first == val)
	  return true;
	++first;
      }
      return false;
    }
  };
/////////////////////////////////////////////////////////////////





/******************************************************************/
struct Node
{
  uint x, y, fScore, gScore; 
  // in order to make priority_queue work, we need to compare fScore. see comp_node for detail.
  bool operator==(const Node &n) const {
    return (x == n.x && y == n.y);
  }
  bool operator!=(const Node &n) const {
    return (x != n.x || y != n.y);
  }
  friend ostream& operator<<(ostream& os, const Node&n) {
    os << '(' << n.x << ", " << n.y << ')';
    return os;
  }
};

/*********************************************************************/
// for priority_queue
struct comp_node_fScore
{
  bool operator()( const Node &a, const Node &b) const{
    return a.fScore > b.fScore;
  }
};

struct comp_node_gScore
{
  bool operator()( const Node &a, const Node &b) const{
    return a.gScore > b.gScore;
  }
};

/*********************************************************************/
// creat hash key function for Node Class, so that unordered_map can work.
namespace std
{
  template<> struct hash<Node>
  {
    std::size_t operator()(const Node& n) const {
      std::size_t const h1 (std::hash<std::size_t>{}(n.x) );
      std::size_t const h2 (std::hash<std::size_t>{}(n.y) );
      return h1 ^ (h2 << 1);
    }
  };
}

/*********************************************************************/
// priority queue specialized for gScore( lowest on top )
// gScore is the true distance from start to goal
using p_queue =  prioriyQueue<Node, std::vector<Node>, comp_node_gScore>;




#endif
