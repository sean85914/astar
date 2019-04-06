#include <iostream>
#include <cstdlib>
#include <ctime>
#include "opencv2/core.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"
#include "astar.hpp"

int height, width; // Image dimension

static int count = 0;
int start_x, start_y, end_x, end_y;
bool get_in = false;
bool ASTAR_VERBOSE = true;

void mouse_cb(int Event, int x, int y, int flags, void* param)
{
  
  if(Event == CV_EVENT_LBUTTONUP){
    if(count==0){
      start_x = y; start_y = x; ++count;
      printf("Start point set at (%d, %d)\n", y, x);
    }
    else{
      end_x = y; end_y = x;
      printf("End point set at (%d, %d)\n", y, x);
      get_in = true;
    }
  }
}

int main(int argc, char** argv)
{
  if(argc!=3) {printf("Not enough inputs.\n"); return 0;}
  height = atoi(argv[1]);
  width = atoi(argv[2]);
  
  srand(time(NULL));
  cv::Mat img(height, width, CV_8UC3);
  for(int i=0; i<img.rows; ++i){
    for(int j=0; j<img.cols; ++j){
      double x = (double) rand() / (RAND_MAX +1.0);
      if(x>=0.2){
        img.at<cv::Vec3b>(i, j)[0] = 255.;
        img.at<cv::Vec3b>(i, j)[1] = 255.;
        img.at<cv::Vec3b>(i, j)[2] = 255.;
      }
      else{
        img.at<cv::Vec3b>(i, j)[0] = 0.;
        img.at<cv::Vec3b>(i, j)[1] = 0.;
        img.at<cv::Vec3b>(i, j)[2] = 0.;
      }
    }
  }
  // Convert img to Eigen::MatrixXd
  Eigen::MatrixXd map(height, width);
  for(int i=0; i<img.rows; ++i){
    for(int j=0; j<img.cols; ++j){
      if(img.at<cv::Vec3b>(i, j)[0] == 255.) map(i, j)=0.;
      else map(i, j)=100.;
    }
  }
  cv::namedWindow("Test", CV_WINDOW_NORMAL);
  //cv::resizeWindow("Test");
  cv::setMouseCallback("Test", mouse_cb, NULL);
  cv::imshow("Test", img);
  cv::waitKey(0);
  while(!get_in) {} // Wait to get information
  AStar::Node start(start_x, start_y, NULL), end(end_x, end_y, NULL);
  AStar::AStar planner(start, end, map);
  printf("Start: (%.f, %.f) with value %.f\n", start.get_x(), start.get_y(), map(start.get_x(), start.get_y()));
  printf("End: (%.f, %.f) with value %.f\n", end.get_x(), end.get_y(), map(end.get_x(), end.get_y()));
  AStar::NODE_LIST res; // Planned path placeholder
  if(!planner.plan(res)) {printf("Cannot find path.\n"); return 0;}
  printf("%d\n", (int)res.size());
  for(AStar::NODE_LIST::iterator it=res.begin(); it!=res.end(); ++it){
    img.at<cv::Vec3b>(it->get_x(), it->get_y())[0] = 0.; // B
    img.at<cv::Vec3b>(it->get_x(), it->get_y())[1] = 0.; // G
    img.at<cv::Vec3b>(it->get_x(), it->get_y())[2] = 255.; // R
    cv::destroyWindow("Test");cv::namedWindow("Test", CV_WINDOW_NORMAL);
    cv::imshow("Test", img);
    if(it!=std::prev(res.end())) cv::waitKey(100); else cv::waitKey(0);
  }
  return 0;
}
