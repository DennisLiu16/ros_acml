/*include*/
/*for ros related*/
#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseStamped.h>
#include <a_star/isReached.h>
/*for vector include*/
#include <iostream>
#include <vector>
#include <deque>
#include <list>
/*other*/
#include <stdio.h>
#include <stdbool.h>
#include <math.h>

/*define*/
#define LOOP_RATE 10
/*Map Related*/
#define XMAP_SIZE 10    //in meter
#define YMAP_SIZE 10    //in meter
#define X_SHIFT -5.0
#define Y_SHIFT -5.0
#define XMIN 0
#define YMIN 0 
#define E_SIZE 3    //Enlarge size
#define XMAX 99
#define YMAX 99
#define EMPTY -128
#define UNKNOWN -1
#define SAFE 0
#define OCCUPIED 100
/*Destination Related*/
#define ALL 0
#define NEXT 1 
/*Print Related*/
#define PRINT_ALL_DESTINATION 0
#define PRINT_A_STAR_PATH 1
/*Node Related*/
#define CENTER YMAX/2*(XMAX+1)+XMAX/2
#define NodeINIT CENTER

// change
/*
    1. read map size and change
    2. 
*/

/** ref doc : 
*  1. A* algorithm implement:https://dev.to/jansonsa/a-star-a-path-finding-c-4a4h
*  2. Youtube Video:https://www.youtube.com/watch?v=icZj67PTFhc
*  3. Sub Once:https://charon-cheung.github.io/2019/01/16/ROS/ROS%20Kinetic%E7%9F%A5%E8%AF%86/%E5%8F%AA%E5%8F%91%E5%B8%83%E5%92%8C%E8%AE%A2%E9%98%85%E4%B8%80%E6%AC%A1%E6%B6%88%E6%81%AF/#%E5%8F%AA%E5%8F%91%E5%B8%83%E4%B8%80%E6%AC%A1%E6%B6%88%E6%81%AF
*  4. STL Vector:https://ithelp.ithome.com.tw/articles/10231601
*  5. github src:https://github.com/OneLoneCoder/videos/blob/master/OneLoneCoder_PathFinding_AStar.cpp
*  6. quaternions:https://quaternions.online/   -> (-pi)~(pi)
*/ 

/**
* This cpp goal
* 1. load map once before a_star
*    -> need to get topic once
w
* 2. enlarge map to fit the car's size
*    -> enlarge 2 pixel if occupied
* 3. use A* algorithm to find the best path
*    -> build open,closed list
*    -> build node structure
*
* 4. send the nodes in path to topic-pose like lab2
*
*/

/** @brief Algorithm Function list
*
*   1. isValid
*       @input:
*       @rtnval:
*       @des:
*
*   2. isDestination
*       @input: 
*       @rtnval: (isDestination) : true ? false
*       @des:
*
*   3. calculate_Heuristic
*       @input: 
*       @rtnval: double , Heristic Function's Val
*       @des:
*
*/

/**
 * @brief Map Function list
 *
 *  1. EnlargeMap
 *      @input: none
 *      @rtnval: none
 *      @des: first,copy data[10000] to map[x][y]
 *            second,enlarge two units if the grid Occupied
 * 
 */

 /**
  * @brief Disguess
  *     # isReached do something to topic
  */

  /*structure df*/
  struct true_Coordinate
  {
      /*unit : meter*/
      float x;
      float y;

      bool operator==(const true_Coordinate& c) const {
        return((c.x == x) && c.y ==y);
      }
  };

  struct grid_Coordinate
  {
      /*unit : grid*/
      int x;
      int y;

      bool operator==(const grid_Coordinate& c) const {
        return((c.x == x) && c.y ==y);
      }
  };
  
  /*for A* algorithm*/
  struct Node
  {
      grid_Coordinate self;
      Node* parent;
      std::vector<Node*> vecNeighbors;  //inc eight neighbor region
      float gCost,fCost;  //g -> Local , f -> Global
      float w,z;
      bool bObstacle;
      bool bVisited;
  };

  /*msg df*/
  nav_msgs::OccupancyGridConstPtr my_map;
  a_star::isReachedConstPtr my_reached; 

  /*vars*/
  int Map[100][100] = {};
  int eMap[100][100] = {};
  //target in 2x4 matrix
  std::deque<true_Coordinate> destination_deque;
  std::deque<Node*> AStar_Path;

  Node *nodes = nullptr;
  Node *nodeStart = nullptr;
  Node *nodeEnd = nullptr;
  
  /*function*/
  void reachCallBack(const a_star::isReachedConstPtr &Reach)
  {
      my_reached = Reach;
  }

  static bool isDestination(int x, int y,bool con)
  {
      switch(con)
      {
          case ALL:
          {
                /*change to meter first*/
                float new_x = (float)(x+1)/XMAP_SIZE + X_SHIFT;   //need to casting to float 
                float new_y = (float)(y+1)/YMAP_SIZE + Y_SHIFT;
                true_Coordinate _current={
                    .x = new_x,
                    .y = new_y
                    
                };
                std::deque<true_Coordinate>::iterator iter = find(destination_deque.begin(),destination_deque.end(),_current);
                /*find in deque*/
                if(iter != destination_deque.end()) 
                    return true;
                return false;
                break;
          }
          case NEXT:
          {
              if(&nodes[y*(XMAX+1) + x] == nodeEnd)
                return true;
              return false;
              break;
          }

      }
        
  }

  static bool isInPath(int x, int y)
  {
      std::deque<Node*>::iterator iter = find(AStar_Path.begin(),AStar_Path.end(),&nodes[y*(XMAX+1) + x]);
      if(iter != AStar_Path.end())
        return true;
      return false;
  }

  void print(int type)
  {
    switch(type)
    {
    case PRINT_ALL_DESTINATION:
        printf("Map\n");
        for (int y = YMAX; y > YMIN-1 ; y--)
        {
            for (int x = XMIN; x < XMAX +1; x++)
            {
                if(Map[x][y] == OCCUPIED)
                    printf("+");
                else if(Map[x][y] == SAFE)
                    printf(" ");
                else
                    printf(" ");
            }
            printf("\n");
        }
        printf("\n\n");
        printf("eMap\n");

        for (int y = YMAX; y > YMIN-1 ; y--)
        {
            for (int x = XMIN; x < XMAX +1; x++)
            {
                if(isDestination(x,y,ALL))
                    printf("o");
                else if(eMap[x][y] == OCCUPIED)
                    printf("+");
                else if(eMap[x][y] == SAFE)
                    printf(" ");
                else
                    printf(" ");
            }
            printf("\n");
        }
        break;
    
    case PRINT_A_STAR_PATH:
        printf("A Star Path\n");
        for(int y = YMAX; y > YMIN-1; y--)
        {
            for(int x = XMIN; x < XMAX+1; x++)
            {
                if(isInPath(x,y))
                    printf("o");
                else if(eMap[x][y] == OCCUPIED)
                    printf("+");
                else if(eMap[x][y] == SAFE)
                    printf(" ");
                else
                    printf(" ");
            }
            printf("\n");
        }
        break;
    }
  }

  void EnlargeMap()
  {
    // confirm navigation_sim working
    
    ROS_INFO("Start Map Update");
    printf("%d\n",my_map->info.height*my_map->info.width);
    /*make 2D map and init eMap*/
    for (size_t i = XMIN; i < XMAX+1; i++)
    {
        for (size_t j = XMIN; j < YMAX+1; j++)
        {
            Map[i][j] = my_map->data[(XMAX+1)*j+i];
            eMap[i][j] = EMPTY;
        }
    }

    /*enlarge obstacles to fit phsical car size*/ 
    for (size_t i = XMIN; i <= XMAX; i++)
    {
        for (size_t j = XMIN; j <= YMAX; j++)
        {
            /*if occupied then enlarge*/
            if(Map[i][j] == OCCUPIED)
            {
                for(int x_bias = -E_SIZE;x_bias < E_SIZE+1;x_bias++)
                {
                    for(int y_bias = -E_SIZE;y_bias <E_SIZE+1;y_bias++)
                    {
                        /*if in range*/
                        if(i+x_bias >= 0 && i+x_bias <= XMAX && j+y_bias >=0 && j+y_bias <= YMAX)
                        {
                            eMap[i+x_bias][j+y_bias] = OCCUPIED;
                        }
                    }
                }
            }
            /*if eMap empty*/
            else if(eMap[i][j] == EMPTY)
            {
                /*if nothing or unknown then copy*/
                if(Map[i][j] == SAFE || Map[i][j] == UNKNOWN)
                {
                    eMap[i][j] = Map[i][j];
                }
            }  
        } 
    }
    /*print all destinations*/
    print(PRINT_ALL_DESTINATION);
    ROS_INFO("Finish Map Update");
  }

  void InitDestination()
  {
      /*Init Target Coordinate*/
      true_Coordinate target_1={
          .x = 3.00,
          .y = 4.00
      };

      true_Coordinate target_2={
          .x = 3.00,
          .y = -2.00
      };

      true_Coordinate target_3={
          .x = -4.00,
          .y = -3.00
      };

      true_Coordinate target_4={
          .x = -4.00,
          .y = 4.00
      };

      destination_deque.push_back(target_1);
      destination_deque.push_back(target_2);
      destination_deque.push_back(target_3);
      destination_deque.push_back(target_4);
  }

  bool updateNodeEnd()
  {
      if(!destination_deque.empty())
      {
          true_Coordinate end;
          end = destination_deque.front();
          destination_deque.pop_front();
          /*true coordinate to grid coordinate*/
          end.x += (float)XMAP_SIZE/2;  //shift (0.0,0.0)
          end.y += (float)YMAP_SIZE/2;
          size_t x = (size_t)end.x * XMAP_SIZE -1;
          size_t y = (size_t)end.y * YMAP_SIZE -1;
          ROS_INFO("next destination(%lu,%lu)\n",x,y);
          nodeEnd = &nodes[y*(XMAX + 1) + x];
          return true;
      }
      else
        ROS_WARN("destination queue is empty while updating\n");
      return false;
  }

  bool InitNodes(size_t start_place)
  {
      printf("Enter InitNodes");
       /*Init All Nodes*/
      nodes = new Node[(XMAX+1)*(YMAX+1)];  //build all nodes for eMap
      /*Init nodes val*/
      for(int x = 0; x < XMAX+1; x++)
      {
        for(int y = 0; y < YMAX+1; y++)
        {
            nodes[y*(XMAX+1)+x].self.x = x;
            nodes[y*(XMAX+1)+x].self.y = y;
            nodes[y*(XMAX+1)+x].parent = nullptr;
            nodes[y*(XMAX+1)+x].bVisited = false;
            nodes[y*(XMAX+1)+x].gCost = INFINITY;
            nodes[y*(XMAX+1)+x].fCost = INFINITY;
            if(eMap[x][y] == SAFE)
                nodes[y*(XMAX+1)+x].bObstacle = false;
            else
                nodes[y*(XMAX+1)+x].bObstacle = true;
        }
      }
      /*Init neighbors*/
      for(int x = 0; x < XMAX+1; x++)
      {
          for(int y = 0;y < YMAX+1; y++)
          {
             /*NSWE*/
              if(y>0)
                nodes[y*(XMAX+1)+x].vecNeighbors.push_back(&nodes[(y - 1)*(XMAX + 1) + (x + 0)]);
              if(y<YMAX)
                nodes[y*(XMAX+1)+x].vecNeighbors.push_back(&nodes[(y + 1)*(XMAX + 1) + (x + 0)]);
              if(x>0)
                nodes[y*(XMAX+1)+x].vecNeighbors.push_back(&nodes[(y + 0)*(XMAX + 1) + (x - 1)]);
              if(x<XMAX)
                nodes[y*(XMAX+1)+x].vecNeighbors.push_back(&nodes[(y + 0)*(XMAX + 1) + (x + 1)]);

             /*Diagonally*/
              if(x>0 && y>0)
                nodes[y*(XMAX+1)+x].vecNeighbors.push_back(&nodes[(y - 1)*(XMAX + 1) + (x - 1)]);
              if(y<YMAX && x>0)
                nodes[y*(XMAX+1)+x].vecNeighbors.push_back(&nodes[(y + 1)*(XMAX + 1) + (x - 1)]);
              if(x<XMAX && y>0)
                nodes[y*(XMAX+1)+x].vecNeighbors.push_back(&nodes[(y - 1)*(XMAX + 1) + (x + 1)]);
              if(x<XMAX && y<YMAX)
                nodes[y*(XMAX+1)+x].vecNeighbors.push_back(&nodes[(y + 1)*(XMAX + 1) + (x + 1)]);
          }
      }
      /*Init Start at center*/
      nodeStart = &nodes[start_place];
      /*Init first nodeEnd to target_1*/
      return updateNodeEnd();
  }

  void Direction_Handler(Node* _current)
  {
      int x = _current->self.x;
      int y = _current->self.y;
      int p_x = _current->parent->self.x;
      int p_y = _current->parent->self.y;

      float theta = atan2((float)y-p_y,(float)x-p_x);
      //printf("%d,%d,%f\n",p_x,p_y,theta/3.14*180.0);
      _current->parent->w = cos(theta/2);
      _current->parent->z = sin(theta/2);
  }

  bool Solve_AStar(Node* start_node)
  {
      /*confirm Node is not nullptr*/
      size_t start_place;
      if(start_node == nullptr)
        start_place = NodeINIT;
      else
        start_place= (size_t)(start_node->self.y)*(XMAX+1)+start_node->self.x;

      bool ret = InitNodes(start_place);

      if(ret)
      {
        ROS_INFO("Start Solve A Star");
        auto distance = [](Node* a, Node* b)
        {
            return sqrtf((a->self.x - b->self.x)*(a->self.x - b->self.x)+(a->self.y - b->self.y)*(a->self.y - b->self.y));
        };

        auto heuristic = [distance](Node* a, Node* b)
        {
            return distance(a,b);
        };
        
        /*init nodeCurrent and update fcost and gcost*/
        Node* nodeCurrent = nodeStart;
        nodeStart->gCost = 0.0f;
        nodeStart->fCost = heuristic(nodeStart,nodeEnd);

        /*init NotTestList and put nodeStart in it*/
        std::list<Node*> listNotTestedNodes;
        listNotTestedNodes.push_back(nodeStart);

        /*A* Main*/
        while(!listNotTestedNodes.empty() && nodeCurrent != nodeEnd)
        {
            listNotTestedNodes.sort([](const Node* lhs, const Node* rhs){return lhs->fCost < rhs->fCost;});

            /*ditch visited nodes*/
            while(!listNotTestedNodes.empty() && listNotTestedNodes.front()->bVisited)
                listNotTestedNodes.pop_front();
            
            /*ensure list not empty after ditched some node*/
            if(listNotTestedNodes.empty())
                break;
            
            nodeCurrent = listNotTestedNodes.front();
            nodeCurrent->bVisited = true;

            /*check neighbors*/
            for(auto nodeNeighbor : nodeCurrent->vecNeighbors)
            {
                /*add nodeNeighbor to NotTestedList only if not visited and not obstacle*/
                if(!nodeNeighbor->bVisited && nodeNeighbor->bObstacle == SAFE)
                    listNotTestedNodes.push_back(nodeNeighbor);

                /*compare gcost of neighbors to decide nodeCurrent become parent of it's neighbors or not*/
                float fPossiblyLowerGoal = nodeCurrent->gCost + distance(nodeCurrent,nodeNeighbor);  
                if(fPossiblyLowerGoal < nodeNeighbor->gCost)
                {
                    nodeNeighbor->parent = nodeCurrent;
                    nodeNeighbor->gCost = fPossiblyLowerGoal;
                    nodeNeighbor->fCost = nodeNeighbor->gCost + heuristic(nodeNeighbor, nodeEnd);
                }
            }
        }
        /*Store Path in Deque*/
        if(nodeEnd != nullptr)
        {
            Node* nodeCurrent = nodeEnd;
            
            while(nodeCurrent->parent != nullptr)
            {
                Direction_Handler(nodeCurrent);
                AStar_Path.push_front(nodeCurrent);
                nodeCurrent = nodeCurrent->parent;
            }
        }       
        print(PRINT_A_STAR_PATH);
        ROS_INFO("A Star Finished");
      }
      return ret;
  }

  int main(int argc, char** argv)
  {
      ros::init(argc,argv,"A_star_Sim");
      ros::NodeHandle nh;
      /*get topic once*/
      my_map = ros::topic::waitForMessage<nav_msgs::OccupancyGrid>("/map",nh,ros::Duration(5));
      ros::Publisher goal_pub = nh.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal",1000);
      ros::Subscriber reach_sub = nh.subscribe("/isReached",10,reachCallBack);
      ros::Rate rate(LOOP_RATE);
      sleep(1);

      /* Enlarge the map */
      if(my_map!=nullptr)
      {
        InitDestination();  
        EnlargeMap();
        bool navigationState = false;   /*in navigation*/
        int count = 0;
        while(ros::ok())
        {
            if(!navigationState)
            {
            /*update A Star*/
                /*first in will be nullptr*/
                bool ret = Solve_AStar(nodeEnd);
                /*No more Destination*/
                if(!ret)
                    break;
                /*go to nav state*/
                else
                    navigationState = true;
            }
            else
            {
                /*update topic*/
                if(my_reached->Reached || count == 0)
                {
                    /*check remain element is Destination or not*/
                    if(AStar_Path.front() == nodeEnd)
                    {
                        /*update navigationState and add nodeEnd's w, z*/
                        navigationState = false;
                        count = 0;
                        AStar_Path.front()->w = AStar_Path.front()->parent->w;
                        AStar_Path.front()->z = AStar_Path.front()->parent->z;
                    }
                    /*pub next*/
                    true_Coordinate f;
                    geometry_msgs::PoseStamped my_robot_goal;
                    Node* nodeNext = AStar_Path.front();
                    f.x = (float)(nodeNext->self.x+1)/XMAP_SIZE + X_SHIFT;
                    f.y = (float)(nodeNext->self.y+1)/YMAP_SIZE + Y_SHIFT;
                    
                    my_robot_goal.pose.position.x = f.x;
                    my_robot_goal.pose.position.y = f.y;
                    my_robot_goal.pose.orientation.w = nodeNext->w;
                    my_robot_goal.pose.orientation.z = nodeNext->z;

                    goal_pub.publish(my_robot_goal);
                    float tmp = atan2(2*(nodeNext->w*nodeNext->z),1-2*(nodeNext->z*nodeNext->z));
                    printf("%d,%d,%f\n",AStar_Path.front()->self.x,AStar_Path.front()->self.y,tmp/3.14*180);
                    AStar_Path.pop_front();
                    count++;
                }
                  
            }
            ros::spinOnce();
            rate.sleep();      
        }
      }
      else
        ROS_INFO("Didn't get anything from /map");
  }