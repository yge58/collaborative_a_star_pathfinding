
CC = g++
CXXFLAGS = -c -g -std=c++11
INC = -I./include
SRC = ./src/
BIN = ./bin/
OBJS = main.o Agent.o Map.o



coop_astar: $(OBJS)
	$(CC) $(OBJS) -o $(BIN)coop_astar

main.o: main.cpp
	$(CC) $(CXXFLAGS) $(INC) main.cpp

Agent.o: ./src/Agent.cxx
	$(CC) $(CXXFLAGS) $(INC) $(SRC)Agent.cxx

Map.o:	./src/Map.cxx 
	$(CC) $(CXXFLAGS) $(INC) $(SRC)Map.cxx

clean:
	rm -f core $(OBJS) 
