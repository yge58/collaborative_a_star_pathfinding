/*********************************************************************************************
Copyright <2017> <Yan Ge>

Permission is hereby granted, free of charge, to any person obtaining a copy of this software 
and associated documentation files (the "Software"), to deal in the Software without restriction, 
including without limitation the rights to use, copy, modify, merge, publish, distribute, 
sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is 
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or 
substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING 
BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, 
DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, 
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
/*********************************************************************************************/


/*********************************************************************************************

  Collaborative A-star pathfinding 
  
  based on windowed hierarchical cooperative A-star from David Silver

  (this file contains all.)

  author: Yan Ge 
 
  thank numerous online resource on pathfinding.
  thank David Silver, and his paper "Cooperative Pathfinding".
  thank Professor Hamada Ghenniwa for his teaching.
  thank my mom and dad for supporting my grad study.
  thank myself for not giving up.

  compile:  g++ -g -std=c++11 -fopenmp coop_astar.cpp -o coop_astar
  
  editor:  emacs

/*********************************************************************************************/

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




uint n_agents = 0;



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

/*********************************************************************/
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
/*********************************************************************/



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


/*********************************************************************/
// agents ignore others. basic A-star.
void Agent::set_whole_path()
{
  Node current = start;
  path = {current};
  while (cameFrom.find(current) != cameFrom.end())
    {
      current = cameFrom[current];
      path.push_back(current);
      if (current == goal)
	break;
    }
}


/*********************************************************************/
// agents will take account of other agents' existing path.
void Agent::set_portion_path (const vector<unordered_map<Node, Agent*>>& space_map)
{
  portion_path.clear();
  
  uint steps_left = WINDOW_SIZE;
  Node current = current_node;
  Node prev = current_node;
  Node next_best = current;
  
  if (current == goal) {
    portion_path = {goal, goal, goal, goal, goal, goal, goal, goal};
    return;
  }
  for (auto i = 0; i < WINDOW_SIZE; ++i)
    {
      if (current == goal)
	break;
      
      if (cameFrom.find(current) == cameFrom.end())
	break;

      next_best = cameFrom[current];	

      // CASE 1:  next best node is not occupied, but is it safe for me to go there? lets check collision.
      if (space_map[i].find(next_best) == space_map[i+1].end())
	{
	  cout << "Agent::set_portion_path: at time " << i << " " << name 
	       << " is walking towards " << next_best << endl;
	  cout << "is there an agent walking towards me.  \n";
	  auto search1 = space_map[i].find(current);		
	  if ( search1 != space_map[i].end()) 
	    {
	      cout << "who? ";
	      Agent*  another_agent = search1->second;
	      cout << "Agent " << another_agent->getName() << " is walking towards me!!!\n";
	      cout << "am I walking towards Agent " << another_agent->getName() << " ?\n"; 
        
	      auto search2 = space_map[i-1].find(next_best);
	      if (search1->second == search2->second)
		{
		  cout << "Yes, you are!\n";
		  cout << "pleanse find another next best node other than " << next_best << endl;

		  uint x = current.x;
		  uint y = current.y;
		  cout << "current: " << current << endl;
		  cout << "prev: " << prev << endl;
	  
		  vector<Node> neighbors = {{x-1, y},{x+1, y},{x, y-1},{x, y+1}}; 
		  uint  current_best_gScore = gScore[current]; // gScore, i.e the true distance to goal.
		  uint next_best_gScore = MAX;
	    
		  for (auto& n : neighbors)
		    {
		      if (n == next_best)   
			continue; // continue becasue, next_best_node is to be avoided..
		      if (n == prev)
			continue; //  we dont want agent go back. sometimes going back is the only option.
		      cout << "evaluating: " << n <<  "\tgScore: " << gScore[n] << endl;
		      if ( gScore[n] < next_best_gScore && space_map[i].find(n) == space_map[i].end() ) // find best gScore and it is not in space_map
			{
			  next_best = n;
			  next_best_gScore = gScore[n];
			}
		    }
		  if (next_best_gScore == MAX) // OK, now we consider going back!
		    next_best = prev;	    
		}	
	    }
	  prev = current;
	  current = next_best;
	  portion_path.push_back(current);
	  steps_left--;
	  this->prev_node = prev; // update previous node.
	  this->current_node = current; // update current node;
	  continue;
	}
      
     
      // CASE 2:  next best node is occupied. FIND ANOTHER NODE. 
      cout << "set_portion_path: find 2nd best option\n";
      // get neighbors who are not obstacles
      uint x = current.x;
      uint y = current.y;
      cout << "current: " << current << endl;
      cout << "prev: " << prev << endl;
	  
      vector<Node> neighbors = {{x-1, y},{x+1, y},{x, y-1},{x, y+1}}; 
      uint  current_best_gScore = gScore[current]; // gScore, i.e the true distance to goal.
      uint next_best_gScore = MAX;
	    
      for (auto& n : neighbors)
	{
	  if (n == next_best)   
	    continue; // continue becasue, next_best was occupied.
	       
	  if (n == prev)
	    continue; //  we dont want agent go back. ?? sometimes going back is the only option???

	  cout << "evaluating: " << n <<  "\tgScore: " << gScore[n] << endl;
	  if (gScore[n] < next_best_gScore &&           
	      space_map[i].find(n) == space_map[i].end()) // find best gScore and it is not in space_map
	    {
	      next_best = n;
	      next_best_gScore = gScore[n];
	    }
	}
	    
      // fScore never updated, no neighbor except cameFrom which is occupied.
      // the next best is to stay at current. ??? or going back?
      if (next_best_gScore == MAX)
	{
	  break;
	}
      prev = current;
      current = next_best;
      portion_path.push_back(current);
      steps_left--;	    
        
      this->prev_node = prev; // update previous node.
      this->current_node = current; // update current node;      

    }	// end of for (auto i = 0; i < WINDOW_SIZE; ++i)
  
  while (steps_left--)
    {
      //prev_node = current_node;
      portion_path.push_back(current);
    }
}




/*********************************************************************/
void Agent::getPath(list<Node>& p)
{
  p.clear();
  for (auto& i : path)
      p.push_back(i);
}


void Agent::get_portion_path (list<Node>& p)
{
  p.clear();
  for (auto& i : portion_path)
    p.push_back(i);
}


/*********************************************************************/
void Agent::print_path()
{
  for (const auto& i : path)
    if (i == path.back())
      cout << i << endl;
    else
      cout << i << "->";
}

/*********************************************************************/
bool Agent::getNextNode(const Node& n1, Node& n2)
{
  if (n1 == path.back()){
    cout << n1 << " is the last node on the path. No next node.\n";
    n2 = {MAX, MAX};
    return false;
  }
  int found = 0;
  for( auto& i : path) {
    if(found == 1) {
	n2 = i;
	return true;
      }
    if( i == n1)
      found = 1;
  }
  return false;
}

/********************************************************************/
void Agent::insert_path_after_front( const list<Node>& p)
{
  auto path_after_front = path.begin();
  path.insert(++path_after_front, p.begin(), p.end());
}

/********************************************************************/
void Agent::set_prev_node()
{
  prev_node = get_front_node();
}

/*********************************************************************/
void Agent::pop_front_node()
{
  if (!path.empty())
    path.pop_front();
}


/*********************************************************************/

using hash_map = unordered_map<Node, vector<Agent*>>;

/*********************************************************************/
// Map Class                                             
class Map
{
  // private methods.
  void allocateMap();
  void allocateMap(uint obsPercent); // allocate obstacles according to percentage set by user.

public:
  Map(uint w, uint h); // no obstacles
  Map(uint w, uint h, uint obsPercent); // has obstacles
  Map(const Map& map); 
 
  uint getHeight() const {return height;};
  uint getWidth() const {return width;};
  uint getPixelCount() const {return pixelCount;};
  char** getPtrMapData() const {return ptrMapData;}; 
  void setObstacle(const uint w, const uint h);
  void setObstacle(const vector<pair<uint, uint>>& p);
  void getNeighbors(const Node& n, vector<Node>& Neighbors);
  void get_clean_neighbors (const Node& node, vector<Node>& nodes);
  bool isObstacle(const Node& n);
  void updateMap(const vector<Agent>& agent_list);
  void get_obstacles(unordered_map<Node, char>& m) {m = obstacles;};
  void printMap();
  void print_map_with_agents (const unordered_map<Node, Agent*>& map);
  void printAgentPath(const Agent& agent, const vector<Node>& path);
  void printSuperMap(const uint& time);

private:
  uint width, height;
  uint pixelCount;
  uint obstaclesPercent;
  char** ptrMapData;
  unordered_map<Node, char> obstacles;
};

/*********************************************************************/
Map::Map(uint w, uint h)
{
  width = w + 2; // add board width  
  height = h + 2; 
  pixelCount = width * height;
  allocateMap();
}

/*********************************************************************/
Map::Map(uint w, uint h, uint obsPercent)
{
  width = w + 2;
  height = h + 2; 
  pixelCount = width * height;
  obstaclesPercent = obsPercent;
  allocateMap(obsPercent);
}

/*********************************************************************/
Map::Map( const Map& m)
{
  height = m.getHeight();
  width = m.getWidth();
  pixelCount = m.getHeight() * m.getWidth();
  char** mPtrMapData = m.getPtrMapData();
  
  if(ptrMapData = (char **)malloc( height * sizeof(char*)))
    {
      for (int i = 0; i < height; i++) 
	if (ptrMapData[i] = (char *)malloc( width * sizeof(char)) )
	  for (int j = 0; j < width; j++)
	    ptrMapData[i][j] = mPtrMapData[i][j];
    }
  else
    cerr << "Map Copy error\n"; 
}

/*********************************************************************/
void Map::allocateMap()
{
  if (ptrMapData = (char **)malloc( height * sizeof(char*)))
    for (int i = 0; i < height; i++) {
	if (ptrMapData[i] = (char *)malloc( width * sizeof(char)) )
	  for (int j = 0; j < width; j++) {
	      if (i > 0 && i < height-1 && j > 0 && j < width-1)
		ptrMapData[i][j] = 'O';
	      else
		ptrMapData[i][j] = 'X';
	    }
	else
	  cerr << "Map::allocate error\n";
      }
  else
    cerr << "Map::allocate error: ptrMapData malloc\n"; 
}

/*********************************************************************/
void Map::allocateMap(uint obsPercent)
{
  if (ptrMapData = (char **)malloc( height * sizeof(char*)) )
    for (uint i = 0; i < height; i++)
      if (ptrMapData[i] = (char *)malloc( width * sizeof(char)) )
	for (uint j = 0; j < width; j++) {
	    if (i > 0 && i < height-1 && j > 0 && j < width-1) {		
	      if((rand() % 100) < obsPercent){
		  ptrMapData[i][j] = 'X';
		  obstacles[{j, i}] = 'X';
	      }
		else
		  ptrMapData[i][j] = 'O';
	      }
	    else
	      ptrMapData[i][j] = 'X';
	  }
      else
	cerr << "Map::allocate error\n";
  else
    cerr << "Map::allocate error: ptrMapData malloc\n"; 
}

/*********************************************************************/
void Map::setObstacle(uint x, uint y)
{
  if (x < width && y < height)
    {
      ptrMapData[y][x] = 'X';
      obstacles[{x, y}] = 'X';
    }
}

/*********************************************************************/
void Map::setObstacle(const vector<pair<uint, uint>>& p)
{
  for (auto const&i : p)
    {
      if (i.first < width && i.second < height)
	{
	  ptrMapData[i.second][i.first] = 'X';
	  obstacles[{i.first, i.second}] = 'X';
	}
      else
	cout << "node <" << i.first << ", " << i.second << "> cannot be set!\n";
    }
}


bool Map::isObstacle(const Node &n)
{
  if (ptrMapData[n.y][n.x] == 'X')
    return true;
  else
    return false;
}

/*********************************************************************/
void Map::printMap()
{
  for (uint y = 0; y < height; y++)
    {
      for (uint x = 0; x < width; x++)
	{
	  Node n = {x,y};
	  if (isObstacle(n))
	    {
	      cout << "X\t";
	      continue;
	    }
	  cout << '\t';
	}
      cout << endl;
    }
}

/*********************************************************************/
void Map::print_map_with_agents (const unordered_map<Node, Agent*>& m)
{
  
  for (uint y = 0; y < height; ++y)
    {
      for (uint x = 0; x < width; ++x)
	{
	 Node n = {x,y};
	  if (isObstacle(n))
	    {
	      cout << "X\t";
	      continue;
	    }
	  auto search = m.find(n);
	  if (search != m.end() )
	    { 
	      cout  << search->second->getName() << '\t';
	      continue;
	    }
	  cout << '\t';
	}
      cout << endl;
    }
  cout << endl;
}
/*********************************************************************/
void Map::printAgentPath(const Agent& agent, const vector<Node>& v)
{
  for (Node n : v)
    ptrMapData[n.y][n.x] = agent.getName();
  
  for (int i = 0; i < height; i++) {
    for (int j = 0; j < width; j++)
      cout << ptrMapData[i][j];
    cout << endl;
  }
}

/*********************************************************************/
// if include diagonal nodes, uncomment the code.
void Map::getNeighbors (const Node& n, vector<Node>& Neighbors)
{
  uint x = n.x;
  uint y = n.y;
  Neighbors = {
    // {x-1,y-1,MAX},
    {x , y-1,MAX},
    //  {x+1,y-1,MAX},
    {x-1, y, MAX},
    {x+1, y, MAX},
    //  {x-1,y+1,MAX},
    {x , y+1,MAX},
    //  {x+1,y+1,MAX}
  };
}

/*********************************************************************/
void Map::get_clean_neighbors (const Node& node, vector<Node>& nodes)
{
  uint x = node.x;
  uint y = node.y;
  Node n;
  n = {x, y-1};
  if(!isObstacle(n)) nodes.push_back(move(n));
  n = {x-1, y};
  if(!isObstacle(n)) nodes.push_back(move(n));
  n = {x+1, y};
  if(!isObstacle(n)) nodes.push_back(move(n));
  n = {x, y+1};
  if(!isObstacle(n)) nodes.push_back(move(n));
}

/*********************************************************************/
void Map::updateMap(const vector<Agent>& agent_list)
{
  for (auto const& a : agent_list)
    {
      const auto& n = a.get_current_node(); 
      ptrMapData[n.x][n.y] = a.getName();
    }
}
// MAP CLASS END

/********************************************************************/
// manhattan distance heuristic.
inline uint manhattan_heuristic(const Node& a, const Node& b)
{// the cost moving form one spot to an adjacent spot vertically or horizontally is 10
  return 10 * ( std::abs(static_cast<int>(a.x - b.x)) + std::abs(static_cast<int>(a.y - b.y)) );
}

/********************************************************************/
// Basic A-Star pathfinding for single agent.
bool aStar(Agent &agent,  Map &map)
{
  // empty agent's closedSet
  if (!agent.closedSet.empty())
    agent.closedSet.clear();
  
  if (!agent.openSet.empty())
    cout << "openSet is not empty, error\n";
   
  Node start = agent.getStart();
  Node goal = agent.getGoal();

  // Initially, only the start node is known.
  agent.openSet.push(start);
  // The cost of going from start to start is zero.
  agent.gScore.insert( {start, 0} ) ;
  // For the first node, that value is completely heuristic.
  agent.fScore.insert( {start, manhattan_heuristic(start, goal)} );

  while (!agent.openSet.empty())
    {
      Node current = agent.openSet.top();
      agent.openSet.pop();
      
      if (current == goal)
	return true;

      agent.closedSet.insert({current, agent.fScore[current]}); // change closedSet to vector.
      vector<Node> Neighbors;
      map.getNeighbors(current, Neighbors);

      for (Node& neighbor : Neighbors) {
	  // insert new nodes
	  if(agent.cameFrom.find(neighbor) == agent.cameFrom.end()) {
	      agent.cameFrom.insert(make_pair(neighbor, current));
	      agent.gScore.insert(make_pair(neighbor, MAX));
	      agent.fScore.insert(make_pair(neighbor, MAX));
	    }
	  if (agent.closedSet.find(neighbor) != agent.closedSet.end())
	    continue; // Ignore the neighbor which is already evaluated
       
	  if (map.isObstacle(neighbor)) { // if neighbor is obstacles, add to closedSet.
	      agent.closedSet.insert( {neighbor, MAX} ); // fscore is set to be max.
	      continue;
	  }
	  // The distance from start to a neighbor, diagonal 14, vertical or horizontal 10.
	  uint dist_neighbor = abs((int)(neighbor.x-current.x)) +	\
	    abs((int)(neighbor.y-current.y)) == 2 ? 14:10;
	  
	  uint tentative_gScore = agent.gScore[current] + dist_neighbor;
	  
	  if (tentative_gScore >= agent.gScore[neighbor])
	    continue;
	  
	  agent.cameFrom[neighbor] = current;
	  agent.gScore[neighbor] = tentative_gScore;
	  agent.fScore[neighbor] = tentative_gScore + manhattan_heuristic(neighbor, goal);
	  neighbor.fScore =  tentative_gScore + manhattan_heuristic(neighbor, goal);
	  /*
	    cout << "[" << neighbor.x << ", " << neighbor.y <<"]" <<" is cameFrom " \
	    << "[" << current.x << ", " << current.y << "] ";
	    cout << "gScore: " << agent.gScore[neighbor] << "  fScore: " << agent.fScore[neighbor] << endl << endl;
	  */
	  agent.openSet.push(neighbor);
	}
    }
  return false;
}

/******************************************************************************** 
   reverse resumable A* (Backwards Search ignoring other agents)
   the g value (measured distance to goal) is the true distance heuristic value.
   if g value of requested node is not known, set goal at requestedNode.
*********************************************************************************/
bool get_true_distance_heuristic(Agent& agent, Map& map, const Node& requestNode)
{
  bool goal_found = false;
  
  //check requested Node
  if (map.isObstacle(requestNode))
    {
      cout << "<get_true_distance_heuristic>: Requested node is an obstacle.\n";
      agent.closedSet.insert({requestNode, MAX});
      return false;
    }

  // start position is the goal position.
  Node start = agent.getGoal(); // reversed, goal is start.
  Node goal = requestNode; 
  
  // Initially, only the start node is known.
  agent.openSet.push(start);
  // The cost of going from start to start is zero.
  agent.gScore.insert( {start, 0} ) ;
  // For the first node, that value is completely heuristic.
  agent.fScore.insert( {start, manhattan_heuristic(start, goal)} );

  while (!agent.openSet.empty())
    {
      Node current = agent.openSet.top();
      agent.openSet.pop();
      if (current == goal)
	{
	  goal_found = true;
	}
      agent.closedSet.insert({current, agent.fScore[current]}); // change closedSet to vector.

      vector<Node> Neighbors;
      map.getNeighbors(current, Neighbors);
      for (Node& neighbor : Neighbors) {
	// if neighbors not exist, creat new nodes
	if(agent.cameFrom.find(neighbor) == agent.cameFrom.end()) {
	  agent.cameFrom.insert(make_pair(neighbor, current));
	  agent.gScore.insert(make_pair(neighbor, MAX));
	  agent.fScore.insert(make_pair(neighbor, MAX));
	}
	// Ignore the neighbor which is already evaluated
	if (agent.closedSet.find(neighbor) != agent.closedSet.end())
	  continue; 
	// if neighbor node is obstacle, pust to closedSet and continue.
	if (map.isObstacle(neighbor)) { // if neighbor is obstacles, add to closedSet.
	  agent.closedSet.insert( {neighbor, MAX} ); // fscore is set to be max.
	  continue;
	}
	// if count diagonal neighbor, uncomment this code.
	// uint dist_neighbor = abs((int)(neighbor.x-current.x)) + abs((int)(neighbor.y-current.y)) == 2 ? 14:10;        
	uint dist_neighbor = 10;
	uint tentative_gScore = agent.gScore[current] + dist_neighbor;
	if (tentative_gScore >= agent.gScore[neighbor])
	    continue;
	
	agent.cameFrom[neighbor] = current;
	agent.gScore[neighbor] = tentative_gScore;

	// if goal is already found, manhattan_heuristic is inaccurate to measure fScore.
	// because manhanttan heuristic ignore obstacles.
	// if goal is found, we simply add 20, why? for example, if we know node A's fScore
        // and node B is A's neighbor, and if A is the only parent of B, then the cost of
	// going from A to B is 10, so B_fSCore = A_fSCore + 10;
	// in order for B to reach goal, it must go back to A,
	// so add 10 to B_fScore again, total is 20;
	if (!goal_found)
	  {
	    agent.fScore[neighbor] = tentative_gScore + manhattan_heuristic(neighbor, goal);
	    neighbor.fScore =  tentative_gScore + manhattan_heuristic(neighbor, goal);
	  }
	else
	  {
	    uint newfScore =  agent.fScore[current] + 20;
	    agent.fScore[neighbor] = newfScore;
	    neighbor.fScore = newfScore;
	  }
	/*
	    cout << "[" << neighbor.x << ", " << neighbor.y <<"]" <<" is cameFrom " \
	    << "[" << current.x << ", " << current.y << "] ";
	    cout << "gScore: " << agent.gScore[neighbor] << "  fScore: " << agent.fScore[neighbor] << endl << endl;
	*/
	agent.openSet.push(neighbor);
      }
    }
  return true;
}

/*********************************************************************************/
void print_fScore(const Agent& agent, Map& map)
{
  uint width = map.getWidth();
  uint height = map.getHeight();
  for (uint w = 0; w < width; ++w)
    cout << "X\t";
  cout << endl;
  for (uint h = 1; h < height-1; ++h){
    cout << "X\t";
    for (uint w = 1; w < width-1; ++w){
      Node n{w,h};
      if (map.isObstacle(n)){
	cout << "X\t";
	continue;
      }
      auto search = agent.closedSet.find(n);
      if (search != agent.closedSet.end()){
	if (search->second == MAX)
	  cout << "INF\t";
	else
	  cout << search->second << "\t";
      }
      else
	cout << "0\t";  
    }
    cout << "X\n";
  }
  for (uint w = 0; w < width; ++w)
    cout << "X\t";
  cout << endl;
}

/*********************************************************************************/
void print_gScore(const Agent& agent, Map& map)
{
  uint width = map.getWidth();
  uint height = map.getHeight();
  for (uint w = 0; w < width; ++w)
    cout << "X\t";
  cout << endl;
  for (uint h = 1; h < height-1; ++h){
    cout << "X\t";
    for (uint w = 1; w < width-1; ++w){
      Node n{w,h};
      if (map.isObstacle(n)){
	cout << "X\t";
	continue;
      }
      auto search = agent.gScore.find(n);
      if (search != agent.gScore.end()){
	if (search->second == MAX)
	  cout << "X\t";
	else
	  cout << search->second << "\t";
      }
      else
	cout << "0\t";  
    }
    cout << "X\n";
  }
  for (uint w = 0; w < width; ++w)
    cout << "X\t";
  cout << endl;
}

/********************************************************************************/
void print_hash_map(const hash_map& h)
{
  for (const auto& i : h)
    {
      cout << "Node: " << i.first << " Agent: ";
      for (const auto& j : i.second)
	cout << j->getName() << " ";
      cout << endl;
    }
  cout << endl;
}

/*********************************************************************************/
void print_path(const list<Node> path)
{
  for (const auto& i : path)
    if (i == path.back())
      cout << i << endl;
    else
      cout << i << "->";
}

/*********************************************************************************/
void print_neighbor_nodes(const vector<Node> v)
{
  for(const auto& i : v)
    cout << i << " ";
  cout << endl;
}

/********************************************************************************/
void print_agents_path(const vector<list<Node>>& v)
{
  int n = 0;
  for(const auto& i : v)
    {
      cout << "Agent " << n++ << ": ";
      print_path(i);
    }
}

/*********************************************************************/
uint get_longest_agent_path(const vector<list<Node>>& v)
{
  uint max = 0;
  for (const auto& i : v)
    {
      if (max < i.size())
	max = i.size();
    }
  return max;
}

/*********************************************************************/
void print_conflict_agents(const vector<vector<Agent*>>& conflict_agents_pairs)
{
  cout << "Agents who have conflicts:\n";
  for (const auto& v : conflict_agents_pairs)
    {
      cout << "< ";
      for (const auto& i : v)
	{
	  cout << i->getName() << " ";
	}
      cout << ">\n";
    }
}

/*********************************************************************/
void print_space_map(const vector< unordered_map<Node, Agent*>> &space_map, Map &map)
{
  for (auto i = 0; i < WINDOW_SIZE; i++)
    {
      cout << "\n>>>>>>>>>>>>>    space_map at time [" << i << "]    <<<<<<<<<<<<<<<\n";
      map.print_map_with_agents(space_map[i]);
      cout << endl;
    }
}

/*********************************************************************/
void print_agents_current_position(vector<Agent*> agents)
{
  cout << "agents current position:\n";
  for (const auto& i : agents)
    {
      cout << i->getName() << ": " << i->get_current_node() << endl;
    }
  cout << endl;
}


/*********************************************************************/
// one node, one agent! hash map size shall equal to number of agents.
inline bool check_collision_type1( const hash_map& t0 )
{
  return t0.size() == n_agents;
}


/*********************************************************************/
// check face to face collisions, each colliion involves two agents, where agent1 walking from A to B, agent2 walking from B to A
bool check_collision_type2( const hash_map& t0, hash_map& t1, vector<vector<Node>>& collision_nodes_pairs)
{
  bool conflict_exist = false;
  
  for (const auto& h0 : t0)
    {
      char agent1_name =  h0.second[0]->getName(); // agent 1 is walking from n0 to n0_next;
      Node agent1_n0 = h0.first;
      Node agent1_n1;
      if(!h0.second[0]->getNextNode(agent1_n0, agent1_n1)) 
	{
	  cout << "<detect_collision>:" << agent1_name <<	\
	    ": there is no node after " << agent1_n0 << endl;
	  continue;
	}
      cout << agent1_name <<  " is walking from " << agent1_n0 <<" -> " << agent1_n1 << endl;
      
      for (const auto& h1 : t1) {
	char agent2_name = h1.second[0]->getName();
	if(agent2_name == agent1_name) // if itself, skip.
	  continue;
	
	Node agent2_n1 = h1.first;

	// cout << "Agent " << agent2_name << " at t1: " << agent2_n1 << " ?= " << agent1_n0 << endl;
	if (agent2_n1 == agent1_n0)
	  {
	    Node agent2_n0 = h1.second[0]->get_current_node();
	    //  cout << agent2_n0 << "? === " << agent1_n1 << endl;
	    if (agent2_n0 == agent1_n1)
	      {// voila, c'est tout!
		cout << "we need to stop " << agent2_name << " walking from " << agent2_n0 << " -> " << agent2_n1 << endl;
		vector<Node> nodes_pair { agent1_n0, agent1_n1 };
		collision_nodes_pairs.push_back( nodes_pair );
		conflict_exist = true;
		t1.erase(agent2_n0);
		t1.erase(agent2_n1);
		if (t1.empty())
		  break;
	      }
	  }
      }      
    }
  return conflict_exist;
}



/*********************************************************************/
//agents will not stay still, both move
void fix_pair ( hash_map& t0,  hash_map& t1,  Node a1_current_node,  Node& a2_current_node, Map& m)
{
  bool collision_fixed = false;
  
  Agent* a1 = t0[a1_current_node][0];
  Agent* a2 = t0[a2_current_node][0];
  // gScore is the true distance to goal
  uint a1_current_node_gScore = a1->gScore[a1_current_node];
  uint a2_current_node_gScore = a2->gScore[a2_current_node];

  Node a1_prev_node = a1->getPrevNode();
  Node a2_prev_node = a2->getPrevNode();
  // compare two gScore, if a1 lower, means a1 is closer to goal, a2 try to make way for a1
  cout <<  a1->getName() << ": " << a1_current_node << " gScore: " << a1_current_node_gScore << " prev node: " << a1_prev_node <<endl;
  cout <<  a2->getName() << ": " << a2_current_node << " gScore: " << a2_current_node_gScore << " prev node: " << a2_prev_node <<endl;  
  vector<Node> a1_possible_moves, a2_possible_moves;
  m.get_clean_neighbors(a1_current_node, a1_possible_moves); 
  m.get_clean_neighbors(a2_current_node, a2_possible_moves);
  // lowest gScore on top.
  p_queue a1_gScore_queue;
  p_queue a2_gScore_queue;

  // depend on the cooperative level, neighbors can include more neighborhood, not just adjacent ones.
  cout << "Agent " << a1->getName() << " ----possible next node gScore:\n";
  
  for (auto& a1_next_possible_node : a1_possible_moves) {
      if (a1_next_possible_node == a2_current_node)
	continue; // exclude its original node.

      a1_next_possible_node.gScore = a1->gScore[a1_next_possible_node];
      cout << a1_next_possible_node << " = " << a1->gScore[a1_next_possible_node] << endl;
      a1_gScore_queue.push(a1_next_possible_node);
    }

  cout << "Agent " << a2->getName() << " ----possible next node gScore:\n";
  for ( auto& a2_next_possible_node : a2_possible_moves) {
      if (a2_next_possible_node == a1_current_node)
	continue;// exclude its original next node.
      a2_next_possible_node.gScore = a2->gScore[a2_next_possible_node];
      cout << a2_next_possible_node << " = " << a2->gScore[a2_next_possible_node] << endl; 
      a2_gScore_queue.push(a2_next_possible_node);
    }
  
  // case 1: one agent keep its path, the other changes. this is the best case!
  if (a1_current_node_gScore <= a2_current_node_gScore) // check a2 queue i.e. agent A
    {
      cout << "Agent " << a2->getName() << " gscore checking ... " << endl;
      p_queue q2 = a2_gScore_queue;
      while (!q2.empty()) {
	  Node a2_next_node = q2.top();    
	  if (a2_next_node == a2_prev_node)
	    { // for now, we dont consider going back.
	      q2.pop();
	      continue;
	    }
	  
	  cout << a2_next_node << " gscore = " << a2->gScore[a2_next_node] << endl;;
	  // we hope this is the next node for a2.
	  // but we first check if this node is gona taken by others.
	  if ( t1.find(a2_next_node) != t1.end() )
	    continue;
	  
	  cout << "find fix! inserting node .... " << a2_next_node << endl;
	  list<Node> l2 = {a2_next_node, a2_current_node};
	  cout << "inserting path to front....";
	  print_path(l2);
	  a2->insert_path_after_front( l2 ); // a2 will walk back to its original path
	  collision_fixed = true;
	  break;
      } 
    }
  if (!collision_fixed)
    {
      // a1 changes routes, a2 stays on course., check a1 queue. i.e. agent B
      cout << "Agent " << a1->getName() << " gscore checking ... " << endl;
      p_queue q1 = a1_gScore_queue;
      while(!q1.empty()) {
	Node a1_next_node = q1.top();    
	if (a1_next_node == a1_prev_node) {
	  q1.pop();
	  continue;
	}
	
	cout << a1_next_node << " gscore = " << a1->gScore[a1_next_node] << endl;
	// we hope this is the next node for a2.
	// check if this node is taken by other agents...
	if(t1.find(a1_next_node) != t1.end()) // meaning this node will be occupied by other agents
	  continue;
	
	cout << "find fix! inserting node .... " << a1_next_node << endl;
	// now we got a free node, no other agents are gona take it, so a1 will go to that node temporarily.
	list<Node> l1 = {a1_next_node, a1_current_node}; // we assume a1 will walk back. if there is a collision, we deal with it later...
	cout << "inserting path to front....";
	print_path( l1 );
	a1->insert_path_after_front( l1 );
	cout << "new path is\n";
	a1->print_path();
	collision_fixed = true;
	break;
      }
    }
  
  if (collision_fixed)
    return;
  else
    cout << "type 2 collision not fixed!\n";
 
}

/*********************************************************************/
void fix_agents ( hash_map& hmap_t0,  hash_map& hmap_t1, vector<vector<Node>>& collision_nodes_pairs, Map& m )
{
  cout << "fix_agents now\n";
  uint n_pairs = collision_nodes_pairs.size();
  cout << "n_pairs = " << n_pairs << endl;
  cout << "\n\n";

  for (auto i = 0; i < n_pairs; i++)
    {
      Node n1, n2;
      n1 = collision_nodes_pairs[i][0];
      n2 = collision_nodes_pairs[i][1];
      
      cout << "n1: " << n1 << ", n2: " << n2 << endl;
      fix_pair( hmap_t0, hmap_t1, n1, n2, m );
    }
}

/*********************************************************************/
void update_space_map(vector<unordered_map<Node, Agent*>> &space_map,  Agent* agent)
{
  list<Node> l;
  agent->get_portion_path( l );
  for (auto i = 0; i < WINDOW_SIZE; ++i)
    {
      space_map[i][l.front()] = agent;
      l.pop_front();
    }
}


//////////////////////////////////////////////////////////////////////////////
/* Main */
int main(int args, char **argv)
{
  vector<vector<unordered_map<Node, Agent*>>> space_time_map;
  
  Map map(8, 16, 20);
  // map.setObstacle({{4,1},{4,2},{5,8},{5,3}}); 
  map.printMap();
  cout << endl;
  
  vector<Agent*> agent_list;

  Agent a('a'); a.setStart( {1,1,0} ); a.setGoal( {8, 1, MAX} );  
  Agent b('b'); b.setStart( {8,1,0} ); b.setGoal( {1, 1, MAX} );

  agent_list.push_back(&a);
  if(!get_true_distance_heuristic(a, map, a.getStart()))
    cout << "Agent " << a.getName() << " failed at searching requested location!\n ";
  
   agent_list.push_back(&b);
  if(!get_true_distance_heuristic(b, map, b.getStart()))
    cout << "Agent " << b.getName() << " failed at searching requested location!\n ";

  n_agents = agent_list.size();
  bool all_agents_find_goal = false;

  while(!all_agents_find_goal)
    {
      cout << ">>>>>>>>>>>>>>>>>    Do all agents reach destination?    <<<<<<<<<<<<<<<<\n";
      uint n_agents_at_goal = 0;
      for (auto i = 0; i < n_agents; ++i) 
	if (agent_list[i]->get_current_node() == agent_list[i]->getGoal()) 
	  n_agents_at_goal++;

      if (n_agents_at_goal == n_agents)
	{
	  all_agents_find_goal = true;
	  cout << "Success! All agents reached goal!\n";
	  break;
	}

      vector<unordered_map<Node, Agent*>> space_map(WINDOW_SIZE);
      print_agents_current_position(agent_list);
      for (auto agent : agent_list)
	{
	  agent->set_portion_path( space_map );
	  list<Node> l;
	  agent->get_portion_path( l );
	  print_path(l);
	  update_space_map(space_map, agent);
	}
       print_space_map(space_map, map);
       space_time_map.push_back(space_map);
    }
return 0;
}
//////////////////////////////////////////////////////////////////////////////////////////
