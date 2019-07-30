# AStar Algorithm

AStar algorithm implementation with C++ using **Eigen**.

# File system
- data: sample map data txt file
- include: header
- sample: sample codes
  - `astar.cpp`: read file from input argument and return path from upper left to lower right corner
  - `astar_cv.cpp`: random generate image with size given from input and user can click on the canvas
                  to set the start and end nodes, after press space, the planned path will be saved in
                  a video file with green circles (stands for start and end nodes) and red path.
  - `test_empty.cpp`: test with empty matrix from left-up corner to right-down corner with user specific 
                    matrix size, and output the execution time

# Data structor
  - AStar::Node
    - public members
      - Node(): default constructor
      - Node(double x, double y, Node* parent): constructor with inputs
      - bool operator==: define what is equal of two nodes
      - Node* get_parent(): get parent address of this node
      - double get_f(): get f score
      - double get_g(): get g score
      - double get_h(): get h score
      - void set_f(double f): set f score 
      - void set_g(double g): set g score
      - void set_h(double h): set h score
  - <pre> std::list<AStar::Node> AStar::NODE_LIST </pre>
  
  - AStar
    - public members
      - AStar::AStar(): default constructor
      - AStar::AStar(Node start, Node end, Eigen::MatrixXd map): constructor with inputs
      - void initial(Node start, Node end, Eigen::MatrixXd map): initial inputs for empty constructor
      - bool plan(AStar::NODE_LIST& res): start to plan and place result in res, return true if successfully to find
                                          path from start node to end node and false otherwise
                                          

# Dependencies
* catkin
* CMake
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
![](https://github.com/sean85914/astar/blob/master/src/images/sample_1.png)
and
```
  $ ./src/bin/astar_cv 480 640 0.1
```
A window with random generated image will show up. Click start and end node then press space, the planned result will be saved in current directory with name `demo.avi`
![](https://github.com/sean85914/astar/blob/master/src/images/astar.gif)
