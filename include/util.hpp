#ifndef UTIL_HPP
#define UTIL_HPP

#include "common.hpp"
#include "Agent.hpp"
#include "Map.hpp"

/*********************************************************************/

using hash_map = unordered_map<Node, vector<Agent*>>;


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

#endif
