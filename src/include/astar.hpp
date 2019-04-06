#ifndef ASTAR_HPP_
#define ASTAR_HPP_

/*
   A star path planning implement by c++
   Revised from David's python one
   Editor: Sean Lu 
   Last edited: 4/6
*/

#include <iostream> // std::cerr, std::endl
#include <algorithm> // std::find
#include <list> // std::list
#include <Eigen/Dense> // Eigen::MatrixXd

extern bool ASTAR_VERBOSE;

namespace AStar{
 class Node{
  private:
    double x_, y_;
    double f_, g_, h_; // Cost function
    Node* parent_; // Parent address
  public:
    Node():x_(0.), y_(0.), parent_(NULL), f_(0), g_(0), h_(0){};
    Node(double x, double y, Node* parent):
       x_(x), y_(y), parent_(parent), f_(0), g_(0), h_(0){};
    bool operator==(const Node& target){
      if(x_==target.get_x() && y_==target.get_y()) return true;
      else return false;
    };
    // Get functions
    Node* get_parent(void) {return parent_;}
    double get_f(void) {return f_;}
    double get_g(void) {return g_;}
    double get_h(void) {return h_;}
    double get_x(void) const {return x_;}
    double get_y(void) const {return y_;} 
    // Set functions
    void set_f(double f) {f_ = f;}
    void set_g(double g) {g_ = g;}
    void set_h(double h) {h_ = h;}
  };

 typedef std::list<Node> NODE_LIST;

 class AStar{
   private:
     NODE_LIST planned_list; // Planned list to return
     NODE_LIST open, closed; // Open and closed list
     Node start_, end_; // Start and end node
     Eigen::MatrixXd map_; // Map, in Eigen::MatrixXd
     double height, width; // Dimension of map
     const static int FACTOR=3; // Multiplier to consider as failed planning
     /* Check if given start and goal nodes are valid
        @output
          True if valid, false otherwise
     */
     bool checkIfValid(void){
       // Check bound
       bool bound_check = true, node_check = true;
       if(start_.get_x()<0 or start_.get_x()>=height or start_.get_y()<0 or start_.get_y()>=width) bound_check = false;
       if(end_.get_x()<0 or end_.get_x()>=height or end_.get_y()<0 or end_.get_y()>=width) bound_check = false;
       if(!bound_check) {
         if(ASTAR_VERBOSE)
           std::cerr << "\033[1;31mInvalid input, ignoring...\033[0m" << std::endl; 
         return false;
       }
       // Check nodes
       if(map_(start_.get_x(), start_.get_y()) >= 50.) node_check = false;
       if(map_(end_.get_x(), end_.get_y()) >= 50.) node_check = false;
       if(!node_check) {
         if(ASTAR_VERBOSE) std::cerr << "\033[1;31mStart node or end node at obstacle, ignoring...\033[0m" << std::endl; 
         return false;
       }
       return true;
     }
     void clear_list(void){
       open.clear(); closed.clear(); planned_list.clear();
     }
   public:
     /*
        Empty constructor
     */
     AStar() {}
     /* Constructor:
        @param
          Node start: start node, in form of matrix indice
          Node end: goal node, in form of matrix indice
          Eigen::MatrixXd map: map to plan
     */
     AStar(Node start, Node end, Eigen::MatrixXd map): \
           start_(start), end_(end), map_(map){ 
       height = map.rows(); width = map.cols();
       // Check if valid input
       if(!checkIfValid()) return;
       open.push_back(start_); // Add start node to open list
     }
     /*
        Initial planner
        @param
          Node start
          Node end
          Eigen::MatrixXd 
     */
     void initial(Node start, Node end, Eigen::MatrixXd map){
       start_ = start;
       end_ = end;
       map_ = map;
       height = map.rows(); width = map.cols();
       if(!checkIfValid()) {
         if(ASTAR_VERBOSE)
           std::cerr << "\033[1;31mInvalid input, ignoring...\033[0m" << std::endl;
         return;}
       open.push_back(start_); // Add start node to open list
     }
     /* Planing function
        @param
     /* Planing function
        @param
          NODE_LIST&: planing result placeholder
        @output
          true if successfully find path from start to end, false otherwise
     */  
     bool plan(NODE_LIST &res){
       int count = 0;
       while(open.size()!=0){
         Node* current_node = new Node ();
         count += 1; 
         if(count>=height*FACTOR) {if(ASTAR_VERBOSE) printf("[AStar] Too much iterations, ignore...\n"); 
           clear_list(); return false;}
         // Get the current node
         *current_node = open.front();
         NODE_LIST::iterator current_idx = open.begin();
         for(NODE_LIST::iterator lit=open.begin(); lit!=open.end(); ++lit){
           if(lit->get_f() < current_node->get_f()){
             *current_node = *lit;
             current_idx = lit;
           } // End if
         } // End for
         Node* candidatePtr;
         if(count==1){
           candidatePtr = new Node(current_node->get_x(), current_node->get_y(), NULL);
           candidatePtr->set_g(current_node->get_g());
         }
         else{
           candidatePtr = new Node(current_node->get_x(), current_node->get_y(), candidatePtr);
           candidatePtr->set_g(current_node->get_g());
         }
         // Pop candidate off open list and add to closed list
         open.erase(current_idx);
         closed.push_back(*candidatePtr);
         // Check if found goal
         if(*candidatePtr==end_){
           Node* currentPtr = candidatePtr;
           while(currentPtr!=NULL){
             planned_list.push_back(*currentPtr);
             // Added to make g-value stepwise
             if(currentPtr->get_parent()!=NULL){
               if(currentPtr->get_g() == currentPtr->get_parent()->get_g()+1){
                 currentPtr = currentPtr->get_parent();
               }
               else{
                 double last_g = currentPtr->get_g(); // Save g-value to variable first
                 while(last_g!=currentPtr->get_g()+1) {
                   currentPtr = currentPtr->get_parent();
                 }
               } // End else
             } // End if 
             else break;
           } // End while
           planned_list.reverse(); res = planned_list;
           clear_list();
           return true;
         } // End if
         // --------------------------------------------- Planning ---------------------------------------------
         // Generate children
         NODE_LIST children;
         for(int i=-1; i<2; ++i){
           for(int j=-1; j<2; ++j){
             if(i==0 and j==0) continue;
             double new_x = candidatePtr->get_x()+i,
                    new_y = candidatePtr->get_y()+j;
             // Make sure within map
             if(new_x<0 or new_x>=width or new_y<0 or new_y>=height) continue;
             // Make sure walkable
             if(map_(new_x,new_y) == 100.) continue;
             // Create new node
             Node* new_nodePtr = new Node(new_x, new_y, candidatePtr);
             children.push_back(*new_nodePtr);
           } // End for
         } // End for
         // Loop through children
         NODE_LIST::iterator nit;
         for(nit=children.begin(); nit!=children.end(); ++nit){
           // Check if already in close list
           NODE_LIST::iterator findIter;
           // Create F, G and H values
           nit->set_g(candidatePtr->get_g()+1);
           double dx = nit->get_x() - end_.get_x(),
                  dy = nit->get_y() - end_.get_y();
           nit->set_h(dx*dx+dy*dy);
           nit->set_f(nit->get_g()+nit->get_h()+5*map_(nit->get_x(),nit->get_y()));
           // Last term is added to make path tends to away from potential obstacle
           // Check if already in the open list
           findIter = std::find(open.begin(), open.end(), *nit);
           if(findIter!=open.end()){
             if(findIter->get_g() > nit->get_g()) continue;
           }
           // And child to open list
           open.push_back(*nit);
         } // End for
       } // End while
     } // End plan
  }; // End class
}; // End namespace

#endif
