#include "Agent.hpp"

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
