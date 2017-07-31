#ifndef AGENT_HPP
#define AGENT_HPP


#include "common.hpp"
/*********************************************************************/
// Agent class.
class Agent
{
public:
  Agent(char c) : name(c) { path.clear();};

  bool operator==(const Agent &a) const {
    return (name == a.getName()); 
  }
  
  unordered_map<Node, Node> cameFrom; // <child, parent>
  unordered_map<Node, unsigned int> gScore;
  unordered_map<Node, unsigned int> fScore;
  unordered_map<Node, unsigned int> closedSet;  // calculated fscore nodes
  prioriyQueue<Node, std::vector<Node>, comp_node_fScore> openSet; // lowest fscore on top()

  char getName() const {return name;};
  
  void setStart(const Node& s) {start = s; path.push_back(s); current_node = s;};
  Node getStart() const {return start;};

  void setGoal(const Node& g) {goal = g;};
  Node getGoal() const {return goal;};

  void set_whole_path();  
  void getPath(list<Node> &p);

  void set_portion_path(const vector<unordered_map<Node, Agent*>> &space_map);
  void get_portion_path(list<Node> &p);

  void set_current_node(const Node& n) {if (current_node != n) current_node = n;};
  Node get_current_node() const {return current_node;};

  Node getPrevNode() const {return prev_node;};

  void print_path();

  uint get_path_length() const {return path.size(); };

  void insert_path_after_front( const list<Node>& p);
  bool getNextNode(const Node& n, Node& next);

  void pop_front_node();
  void set_prev_node();
  Node get_front_node() {return path.empty() ? current_node : path.front();};

private:
  char name;
  Node start;
  Node goal;
  Node prev_node;
  Node current_node;
  Node next_node;
  list<Node> path;
  list<Node> portion_path;
};


#endif
