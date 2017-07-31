#ifndef MAP_HPP
#define MAP_HPP

#include "common.hpp"
#include "Agent.hpp"
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
  void get_obstacles(unordered_map<Node, char>& m) {m = obstacles;};
  bool isObstacle(const Node& n);
  
  void getNeighbors(const Node& n, vector<Node>& Neighbors); 
  void get_clean_neighbors (const Node& node, vector<Node>& nodes);

  void updateMap(const vector<Agent>& agent_list);
  
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

#endif
