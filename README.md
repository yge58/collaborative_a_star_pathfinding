# collaborative_a_star_pathfinding
Based on David Silver's paper "Cooperative Pathfinding"

____________________________________________________________
This project is dedicated to my mom and dad!
I want the world to know how grateful I am to be your child!
____________________________________________________________


Author:     Yan Ge

Created from June, 2017 to July, 2017


# compile
No.1 method:

[1]     create a directory, copy all files. type "make", and then "make clean"

[2]     voila! type "cd bin", and type "./coop_astar"


No.2 rogue method

[1]     go to test, copy that one file "collaborative_astar_pathfinding.cpp" into your directory

[2]     g++ -g -std=c++11 collaborative_astar_pathfinding.cpp -o <output>


# I know a-star, what is collaborative a-star?

If you are an expert in multi-agent pathfinding using A-star, please skip.

If you are a beginner of a-star, you may find "A_Pathfinding for Beginners.pdf" in foler "doc"  very helpful. Psudo code from wikipedia on a-star is good too.

Good! Now you are no longer a beginner.

A-star is well-suited for single agent pathfinding! But not for multi-agent pathfinding.

My project is based on David Silver's paper "Cooperative pathfinding".

If you are curious about system design, please find "report_3_system_specification.pdf" and other documentation in doc.


# how to use

In "main.cpp", I believe I have enough comments to get started.


# demo (sorry there is no GUI, command line use only)

// this is a map 8 by 16 with two agents, a and b;
// a wants to go to b's current location, (8,1);
// b wants to go to a's current location, (1,1);

// the following is the starting point.
X	X	X	X	X	X	X	X	X	X	
X	a 		X				b	X	
X									X	
X					X				X	
X								X	X	
X							X		X	
X				X					X	
X	X		X						X	
X					X				X	
X		X			X				X	
X			X						X	
X		X					X		X	
X								X	X	
X									X	
X		X		X	X				X	
X	X							X	X	
X				X					X	
X	X	X	X	X	X	X	X	X	X	

// both agent move, this is what looks at next step.
>>>>>>>>>>>>>    space_map at time [0]    <<<<<<<<<<<<<<<
X	X	X	X	X	X	X	X	X	X	
X		a		X			b		X	
X									X	
X					X				X	
X								X	X	
X							X		X	
X				X					X	
X	X		X						X	
X					X				X	
X		X			X				X	
X			X						X	
X		X					X		X	
X								X	X	
X									X	
X		X		X	X				X	
X	X							X	X	
X				X					X	
X	X	X	X	X	X	X	X	X	X	

//////////////////////////////////////////////////////////
>>>>>>>>>>>>>    space_map at time [1]    <<<<<<<<<<<<<<<

X	X	X	X	X	X	X	X	X	X	
X			a	X		b			X	
X									X	
X					X				X	
X								X	X	
X							X		X	
X				X					X	
X	X		X						X	
X					X				X	
X		X			X				X	
X			X						X	
X		X					X		X	
X								X	X	
X									X	
X		X		X	X				X	
X	X							X	X	
X				X					X	
X	X	X	X	X	X	X	X	X	X	

//////////////////////////////////////////////////////////
>>>>>>>>>>>>>    space_map at time [2]    <<<<<<<<<<<<<<<
X	X	X	X	X	X	X	X	X	X	
X				X	b				X	
X			a						X	
X					X				X	
X								X	X	
X							X		X	
X				X					X	
X	X		X						X	
X					X				X	
X		X			X				X	
X			X						X	
X		X					X		X	
X								X	X	
X									X	
X		X		X	X				X	
X	X							X	X	
X				X					X	
X	X	X	X	X	X	X	X	X	X	

//////////////////////////////////////////////////////////
>>>>>>>>>>>>>    space_map at time [3]    <<<<<<<<<<<<<<<
X	X	X	X	X	X	X	X	X	X	
X				X					X	
X				a	b				X	
X					X				X	
X								X	X	
X							X		X	
X				X					X	
X	X		X						X	
X					X				X	
X		X			X				X	
X			X						X	
X		X					X		X	
X								X	X	
X									X	
X		X		X	X				X	
X	X							X	X	
X				X					X	
X	X	X	X	X	X	X	X	X	X	

//////////////////////////////////////////////////////////
>>>>>>>>>>>>>    space_map at time [4]    <<<<<<<<<<<<<<<
X	X	X	X	X	X	X	X	X	X	
X				X					X	
X					a	b			X	
X					X				X	
X								X	X	
X							X		X	
X				X					X	
X	X		X						X	
X					X				X	
X		X			X				X	
X			X						X	
X		X					X		X	
X								X	X	
X									X	
X		X		X	X				X	
X	X							X	X	
X				X					X	
X	X	X	X	X	X	X	X	X	X	

//////////////////////////////////////////////////////////
>>>>>>>>>>>>>    space_map at time [5]    <<<<<<<<<<<<<<<
X	X	X	X	X	X	X	X	X	X	
X				X	a				X	
X					b				X	
X					X				X	
X								X	X	
X							X		X	
X				X					X	
X	X		X						X	
X					X				X	
X		X			X				X	
X			X						X	
X		X					X		X	
X								X	X	
X									X	
X		X		X	X				X	
X	X							X	X	
X				X					X	
X	X	X	X	X	X	X	X	X	X	

//////////////////////////////////////////////////////////
>>>>>>>>>>>>>    space_map at time [6]    <<<<<<<<<<<<<<<
X	X	X	X	X	X	X	X	X	X	
X				X		a			X	
X				b					X	
X					X				X	
X								X	X	
X							X		X	
X				X					X	
X	X		X						X	
X					X				X	
X		X			X				X	
X			X						X	
X		X					X		X	
X								X	X	
X									X	
X		X		X	X				X	
X	X							X	X	
X				X					X	
X	X	X	X	X	X	X	X	X	X	


//////////////////////////////////////////////////////////
>>>>>>>>>>>>>    space_map at time [7]    <<<<<<<<<<<<<<<
X	X	X	X	X	X	X	X	X	X	
X				X			a		X	
X			b						X	
X					X				X	
X								X	X	
X							X		X	
X				X					X	
X	X		X						X	
X					X				X	
X		X			X				X	
X			X						X	
X		X					X		X	
X								X	X	
X									X	
X		X		X	X				X	
X	X							X	X	
X				X					X	
X	X	X	X	X	X	X	X	X	X	


Now, this window (8 time frames) ends, since both agents didnot reach destination, need another window.
(this windowed approach is from paper "cooperative pathfinding")


>>>>>>>>>>>>>    space_map at time [0]    <<<<<<<<<<<<<<<
X	X	X	X	X	X	X	X	X	X	
X			b	X				a	X	
X									X	
X					X				X	
X								X	X	
X							X		X	
X				X					X	
X	X		X						X	
X					X				X	
X		X			X				X	
X			X						X	
X		X					X		X	
X								X	X	
X									X	
X		X		X	X				X	
X	X							X	X	
X				X					X	
X	X	X	X	X	X	X	X	X	X	



>>>>>>>>>>>>>    space_map at time [1]    <<<<<<<<<<<<<<<
X	X	X	X	X	X	X	X	X	X	
X		b		X				a	X	
X									X	
X					X				X	
X								X	X	
X							X		X	
X				X					X	
X	X		X						X	
X					X				X	
X		X			X				X	
X			X						X	
X		X					X		X	
X								X	X	
X									X	
X		X		X	X				X	
X	X							X	X	
X				X					X	
X	X	X	X	X	X	X	X	X	X	



>>>>>>>>>>>>>    space_map at time [2]    <<<<<<<<<<<<<<<
X	X	X	X	X	X	X	X	X	X	
X	b			X				a	X	
X									X	
X					X				X	
X								X	X	
X							X		X	
X				X					X	
X	X		X						X	
X					X				X	
X		X			X				X	
X			X						X	
X		X					X		X	
X								X	X	
X									X	
X		X		X	X				X	
X	X							X	X	
X				X					X	
X	X	X	X	X	X	X	X	X	X	



>>>>>>>>>>>>>    space_map at time [3]    <<<<<<<<<<<<<<<
X	X	X	X	X	X	X	X	X	X	
X	b			X				a	X	
X									X	
X					X				X	
X								X	X	
X							X		X	
X				X					X	
X	X		X						X	
X					X				X	
X		X			X				X	
X			X						X	
X		X					X		X	
X								X	X	
X									X	
X		X		X	X				X	
X	X							X	X	
X				X					X	
X	X	X	X	X	X	X	X	X	X	



>>>>>>>>>>>>>    space_map at time [4]    <<<<<<<<<<<<<<<
X	X	X	X	X	X	X	X	X	X	
X	b			X				a	X	
X									X	
X					X				X	
X								X	X	
X							X		X	
X				X					X	
X	X		X						X	
X					X				X	
X		X			X				X	
X			X						X	
X		X					X		X	
X								X	X	
X									X	
X		X		X	X				X	
X	X							X	X	
X				X					X	
X	X	X	X	X	X	X	X	X	X	



>>>>>>>>>>>>>    space_map at time [5]    <<<<<<<<<<<<<<<
X	X	X	X	X	X	X	X	X	X	
X	b			X				a	X	
X									X	
X					X				X	
X								X	X	
X							X		X	
X				X					X	
X	X		X						X	
X					X				X	
X		X			X				X	
X			X						X	
X		X					X		X	
X								X	X	
X									X	
X		X		X	X				X	
X	X							X	X	
X				X					X	
X	X	X	X	X	X	X	X	X	X	



>>>>>>>>>>>>>    space_map at time [6]    <<<<<<<<<<<<<<<
X	X	X	X	X	X	X	X	X	X	
X	b			X				a	X	
X									X	
X					X				X	
X								X	X	
X							X		X	
X				X					X	
X	X		X						X	
X					X				X	
X		X			X				X	
X			X						X	
X		X					X		X	
X								X	X	
X									X	
X		X		X	X				X	
X	X							X	X	
X				X					X	
X	X	X	X	X	X	X	X	X	X	



>>>>>>>>>>>>>    space_map at time [7]    <<<<<<<<<<<<<<<
X	X	X	X	X	X	X	X	X	X	
X	b			X				a	X	
X									X	
X					X				X	
X								X	X	
X							X		X	
X				X					X	
X	X		X						X	
X					X				X	
X		X			X				X	
X			X						X	
X		X					X		X	
X								X	X	
X									X	
X		X		X	X				X	
X	X							X	X	
X				X					X	
X	X	X	X	X	X	X	X	X	X	


>>>>>>>>>>>>>>>>>    Do all agents reach destination?    <<<<<<<<<<<<<<<<
Success! All agents reached goal!

# demo end


    

