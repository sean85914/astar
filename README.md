# AStar Algorithm

AStar algorithm implementation with C++ using **Eigen**.

# File system
- data: sample map data txt file
- include: header
- sample: sample codes
  - astar.cpp: read file from input argument and return path from upper left to lower right corner
  - astar_cv.cpp: random generate image with size given from input and user can click on the canvas
                  to set the start and end node, after press space, it will start to draw red path 
                  from user specific start node to end node

# Data structor
  - AStar::Node
    - public members
      - Node(): empty constructor
      - Node(double x, double y, Node* parent): constructor with inputs
      - bool operator==: define what is equal of two nodes
      - Node* get_parent(): get parent address of this node
      - double get_f(): get f score
      - double get_g(): get g score
      - double get_h(): get h score
      - void set_f(double f): set f score 
      - void set_g(double g): set g score
      - void set_h(double h): set h score
  - std::list<AStar::Node> AStar::NODE_LIST 
  
  - AStar
    - public members
      - AStar::AStar(): empty constructor
      - AStar::AStar(Node start, Node end, Eigen::MatrixXd map): constructor with inputs
      - void initial(Node start, Node end, Eigen::MatrixXd map): initial inputs for empty constructor
      - bool plan(AStar::NODE_LIST& res): start to plan and place result in res, return true if successfully to find
                                          path from start node to end node
                                          

# Dependencies
* CMake: at least 3.5
* Eigen3
* OpenCV

# How to use

```
  $ cd && git clone https://github.com/sean85914/astar.git
  $ cd astar && catkin_make
```

Run sample codes by
```
  $ ./src/bin/astar src/data/map.txt
```
and
```
  $ ./src/bin/astar_cv 100 100
```
A window with random generated image will show up. Click start and end node then press space, the window will start to flash and the red path will shown.
