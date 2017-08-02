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
  project created from June to July, 2017.
 
  thank numerous online resource on pathfinding.
  thank David Silver, and his paper "Cooperative Pathfinding".
  thank Professor Hamada Ghenniwa for his teaching.
  thank my mom and dad for supporting my grad study.
  thank myself for not giving up.

  compile:  g++ -g -std=c++11 ./src/Agent.cxx ./src/Map.cxx main.cpp -o main

  editor:  emacs

/*********************************************************************************************/
#include "common.hpp"
#include "Agent.hpp"
#include "Map.hpp"
#include "util.hpp"



//////////////////////////////////////////////////////////////////////////////
/* Main */
int main(int args, char **argv)
{
  vector<vector<unordered_map<Node, Agent*>>> space_time_map;

  /* Create a 8x16 map (width 8, height 16), with 20% random obstacles */
  Map map(8, 16, 20);
  // map.setObstacle({{4,1},{4,2},{5,8},{5,3}});
  /* show the map */
  map.printMap();
  cout << endl;

  vector<Agent*> agent_list;

  /* Agent a, named 'a', start at (1,1), end at (8,1) */
  Agent a('a'); a.setStart( {1,1,0} ); a.setGoal( {8, 1, MAX} );  
  /* Agent b, named 'b', start at (8,1), end at (1,1) */
  Agent b('b'); b.setStart( {8,1,0} ); b.setGoal( {1, 1, MAX} );

  // calculate each agent's true distances to goa in parallel.
  agent_list.push_back(&a);
  if(!get_true_distance_heuristic(a, map, a.getStart()))
    cout << "Agent " << a.getName() << " failed at searching requested location!\n ";
  
  agent_list.push_back(&b);
  if(!get_true_distance_heuristic(b, map, b.getStart()))
    cout << "Agent " << b.getName() << " failed at searching requested location!\n ";

  uint n_agents = agent_list.size();
  bool all_agents_find_goal = false;
  /*have all agents reach goal yet?*/
  while(!all_agents_find_goal)
    {
      cout << ">>>>>>>>>>>>>>>>>    Do all agents reach destination?    <<<<<<<<<<<<<<<<\n";
      uint n_agents_at_goal = 0;
      /*get agent current location, if agent at goal, increment n_agents_at_goal. */
      for (auto i = 0; i < n_agents; ++i) 
	if (agent_list[i]->get_current_node() == agent_list[i]->getGoal()) 
	  n_agents_at_goal++;
      /*compare number of agents at goal, with number of agents.*/
      if (n_agents_at_goal == n_agents)
	{ /*If yes, good news, all agents find goal!*/
	  all_agents_find_goal = true;
	  cout << "Success! All agents reached goal!\n";
	  break;
	}
      /* 99% of computation time is spent from now on...*/
      /* 
	 create a vector of hash map, the key of hash map is Node, value is a pointer to an agent. 
         the window size is 8.
	 Now we just created a windowed space-time map. what is that? let me explain.
	 
	 Time is not time like min or sec. Time is unit to represent a frame, a step for all agents.
	 below, I used a vector. space_time_map[0] means at the begining. 
	 space_time_map[7] means after 7 steps, at step 8 or the end of current window,
	 where are the agents located on the map. 

	 say if at time 5, Yan is located at node (3,4).
	 how to capture Yan? space_time_map[4][{3,4}] ; I hope you got the idea, the syntax may be wrong.

	 notice that, one node, one agent, it is not allowed to have multiple agents sitting at one node. 
	 Now I hope that you know what is space_time_map, and what I mean by time.
      */
      vector<unordered_map<Node, Agent*>> space_time_map(WINDOW_SIZE); 
      /* iterate agent_list, set path for each agent*/
      for (auto agent : agent_list)
	{/* say it is Yan's turn. Yan sets path according to space_time_map */
	  agent->set_portion_path( space_time_map );
	  /* put Yan's path on the space_time_map */
	  update_space_map(space_time_map, agent);
	}
    } // end of the while loop.
  return 0;
}
