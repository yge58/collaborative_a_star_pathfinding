#include "Map.hpp"


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
//////////////////////////////////////////////////////////////////////
