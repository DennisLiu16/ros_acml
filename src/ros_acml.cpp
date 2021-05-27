// ros debug ref : https://blog.csdn.net/lxn9492878lbl/article/details/89490276
/*include*/
/*for ros related*/
#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>  
#include <ros_acml/isReached.h>
#include <ros_acml/acml_goal.h>
/*for vector include*/
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <deque>
#include <list>
/*other*/
#include <stdio.h>
#include <stdbool.h>
#include <math.h>
#include <assert.h>

/*define*/
#define LOOP_RATE 10
/*Map Related*/
#define EMPTY -128
#define UNKNOWN -1
#define SAFE 0
#define ISPATH 1
#define OCCUPIED 100
#define DEFAULT 0.0
#define E_SIZE 3    //Enlarge size
#define PATH_PUB_INTERVAL 0     //  point interval

#define CLOSE 2
#define ARRIVE 1
#define NOTYET 0
/*print type*/
#define PRINT_TYPE_MAP 0 
#define PRINT_TYPE_ASTAR_PATH 1
#define NEW2D(H, W, TYPE) (TYPE **)new2d(H, W, sizeof(TYPE))

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

/*class - acml*/
class ACML{
    public:
        bool getNewGoal = false;
        /*functions*/
        bool updateMapInfoOnce();
        bool updateAStar();
        bool SubPubInit();
        void* new2d(int h, int w, int size);
        void Enlargemap();
        void print(int type);
        void pubIfReached();
        ~ACML();

    protected:
        /*Vars*/
        nav_msgs::OccupancyGridConstPtr my_map = nullptr;
        geometry_msgs::PoseStampedConstPtr my_sub_goal = nullptr;
        ros_acml::isReachedConstPtr my_reached = nullptr; 
        geometry_msgs::TwistConstPtr my_pose = nullptr;
        ros_acml::acml_goal my_pub_goal;
        ros::NodeHandle nh;
        ros::Publisher goal_pub;
        ros::Subscriber goal_sub;
        ros::Subscriber reach_sub;
        ros::Subscriber pose_sub;
        std::deque<true_Coordinate> destination_deque;
        std::deque<Node*> AStar_Path;
        float X_SHIFT = DEFAULT;    //origin shift , record at info/origin/position
        float Y_SHIFT = DEFAULT;
        float XMAP_SIZE = DEFAULT;        //size in meters , record at info/resolution and info/width , info/height
        float YMAP_SIZE = DEFAULT;
        float XMAP_RATIO = DEFAULT;       // grid / true
        float YMAP_RATIO = DEFAULT;
        int width = (int)DEFAULT;
        int height = (int)DEFAULT;
        int XMAX = (int)DEFAULT;
        int XMIN = (int)DEFAULT;
        int YMAX = (int)DEFAULT;
        int YMIN = (int)DEFAULT;
        int8_t** Map = nullptr;
        int8_t** eMap = nullptr;
        Node *nodes = nullptr;
        Node *nodeStart = nullptr;
        Node *nodeEnd = nullptr;
        

        /*functions*/
        grid_Coordinate* t2g(true_Coordinate*);
        true_Coordinate* g2t(grid_Coordinate*);    // coordinate tf between true and grid 
        void reachCallBack(const ros_acml::isReachedConstPtr &Reach);
        void goalCallBack(const geometry_msgs::PoseStampedConstPtr &Goal);
        void poseCallBack(const geometry_msgs::TwistConstPtr &Pose);
        int8_t** getMap2DArray();
        int8_t** getEMap(int8_t**);
        void initNodes();
        void solveAStar(Node *nodeStart, Node *nodeEnd);
        void Direction_Handler(Node* _current);
        bool isInPath(int,int);
        void pub();
        
              
};
ACML::~ACML()
{
    if(Map!=nullptr)
        delete [] Map;
    if(eMap!=nullptr)
        delete [] eMap;
}

void ACML::pub()
{
    Node* nodeNext = AStar_Path.front();
    grid_Coordinate gPub = {
        .x = nodeNext->self.x,
        .y = nodeNext->self.y
    };
    true_Coordinate *tPub = g2t(&gPub);
    my_pub_goal.pose.position.x = tPub->x;
    my_pub_goal.pose.position.y = tPub->y;
    my_pub_goal.pose.orientation.w = nodeNext->w;
    my_pub_goal.pose.orientation.z = nodeNext->z;
    goal_pub.publish(my_pub_goal);
    printf("Next Destination:%f,%f\n",tPub->x,tPub->y);
    AStar_Path.pop_front();
    if(AStar_Path.empty())
        ROS_INFO("Last Goal Pub");

}

void ACML::pubIfReached()
{
    if(!AStar_Path.empty() && my_reached != nullptr && my_reached->Reached != NOTYET)
    {
        pub();
    }
    else if(my_reached != nullptr && my_reached->Reached == ARRIVE)
    {
        ROS_INFO("Arrive!");
    }
}

bool ACML::updateAStar()
{
    /* only update goal */
    /* check it's from AStarPath or user input*/
    // if( my_pub_goal.pose.position.x != my_sub_goal->pose.position.x ||
    //     my_pub_goal.pose.position.y != my_sub_goal->pose.position.y
    //     )
    AStar_Path.clear();
    {
        printf("%f,%f\n",my_pose->linear.x,my_pose->linear.y);
        printf("%f,%f\n",my_sub_goal->pose.position.x,my_sub_goal->pose.position.y);
        /* if user input do solveAStar */
        true_Coordinate tGoal = {
            .x = (float)my_sub_goal->pose.position.x,
            .y = (float)my_sub_goal->pose.position.y
        };
        grid_Coordinate *gGoal = t2g(&tGoal);
        /* init nodes*/
        initNodes();

        /* solve Astar path*/
        true_Coordinate tStart = {
            .x = (float)my_pose->linear.x,
            .y = (float)my_pose->linear.y
        };


        grid_Coordinate *gStart = t2g(&tStart);
        Node *NodeStart = &nodes[gStart->y*width+gStart->x];
        Node *NodeEnd = &nodes[gGoal->y*width+gGoal->x];
        printf("Start(%d,%d)\n",NodeStart->self.x,NodeStart->self.y);
        printf("End(%d,%d)\n",NodeEnd->self.x,NodeEnd->self.y);
        solveAStar(NodeStart,NodeEnd);
        print(PRINT_TYPE_ASTAR_PATH);

        /*pub first*/
        pub();
        printf("A star update finish\n");
        return true;
    }
}

/*class functions implement*/
bool ACML::updateMapInfoOnce()
{
    /* return true if update ok*/
    my_map = ros::topic::waitForMessage<nav_msgs::OccupancyGrid>("/map",nh,ros::Duration(5));
    if(my_map != nullptr)
    {
        X_SHIFT = my_map->info.origin.position.x;
        Y_SHIFT = my_map->info.origin.position.y;
        XMAP_SIZE = my_map->info.resolution*my_map->info.width;
        YMAP_SIZE = my_map->info.resolution*my_map->info.height;
        width = my_map->info.width;
        height = my_map->info.height;
        XMAP_RATIO = width/XMAP_SIZE;    // grid/true
        YMAP_RATIO = height/YMAP_SIZE;
        XMAX = my_map->info.width -1;
        YMAX = my_map->info.height -1;
        // XMIN = YMIN = 0;    change this if you need
    
        return (width > 0 && height > 0);
    }
    return false;
}

bool ACML::SubPubInit()
{
    /* you should check navigation_sim open first */
    goal_pub = nh.advertise<ros_acml::acml_goal>("/ros_acml/goal",1000);

    /*err ref : https://answers.ros.org/question/36200/subscribing-to-topic-throw-compilation-error/ */
    reach_sub = nh.subscribe("/isReached",10,&ACML::reachCallBack,this);
    goal_sub = nh.subscribe("/move_base_simple/goal",1000,&ACML::goalCallBack,this);
    pose_sub = nh.subscribe("/robot_pose",1000,&ACML::poseCallBack,this);
    my_pub_goal.pose.position.x = 1.0;
    goal_pub.publish(my_pub_goal);
    
    return true;
}

void* ACML::new2d(int h, int w, int size)
{
    // ref : https://mropengate.blogspot.com/2015/12/cc-dynamic-2d-arrays-in-c.html
    int i;
    void **p;

    p = (void**)new char[h*sizeof(void*) + h*w*size];
    for(i = 0; i < h; i++)
        p[i] = ((char *)(p + h)) + i*w*size;

    return p;
}

void ACML::reachCallBack(const ros_acml::isReachedConstPtr &Reach)
{
    my_reached = Reach;
}

void ACML::goalCallBack(const geometry_msgs::PoseStampedConstPtr &Goal)
{
    /*get new goal*/
    my_sub_goal = Goal;
    getNewGoal = true;
}

void ACML::poseCallBack(const geometry_msgs::TwistConstPtr &Pose)
{
    my_pose = Pose;
    
}

void ACML::Enlargemap()
{
    Map = getMap2DArray();
    eMap = getEMap(Map);
    print(PRINT_TYPE_MAP);
}

void ACML::print(int type)
{
    /*write into file , map...*/
    std::string path = "/home/dennis/ROS/ROS_CAR/ros_acml.txt";
    std::ofstream newFile;
    switch(type)
    {
        case PRINT_TYPE_MAP:
        {
            if(Map!=nullptr)
            {
                newFile.open(path);
                /*print map*/
                newFile << "print Map" << std::endl;
                for (int y = YMAX; y > YMIN-1 ; y--)
                {
                    std::string row ="";
                    for (int x = XMIN; x < width; x++)
                    {
                        if(Map[y][x] == OCCUPIED)
                            row.append(1,'+');
                        else if(y == YMIN || x == XMIN || y == YMAX || x == XMAX)
                            row.append(1,'+');
                        else if(Map[y][x] == SAFE)
                            row.append(1,' ');
                        else
                            row.append(1,' ');
                    }
                    newFile << row << std::endl;
                }
                printf("print map ok\n");

                newFile << std::endl << std::endl;

                printf("print eMap \n");
                newFile << "print eMap" << std::endl;
                for (int y = YMAX; y > YMIN-1 ; y--)
                {
                    std::string row ="";
                    for (int x = XMIN; x < width; x++)
                    {
                        if(eMap[y][x] == OCCUPIED)
                            row.append(1,'+');
                        else if(y == YMIN || x == XMIN || y == YMAX || x == XMAX)
                            row.append(1,'+');
                        else if(eMap[y][x] == SAFE)
                            row.append(1,' ');
                        else
                            row.append(1,' ');
                    }
                    newFile << row << std::endl;
                }
                
            }
            newFile << std::endl <<std::endl;
            break;
        }
        case PRINT_TYPE_ASTAR_PATH:
        {
            int8_t **Path = NEW2D(height,width,int8_t);
            for (size_t y = YMIN; y < height; y++)
            {
                for (size_t x = XMIN; x < width; x++)
                {
                    if(isInPath(x,y))
                        Path[y][x] = ISPATH;
                    else
                        Path[y][x] = eMap[y][x];
                }

            }
            newFile.open(path,std::ios::app);
            newFile << "print A Star Path" << std::endl;

            for (int y = YMAX; y > YMIN-1 ; y--)
            {
                std::string row ="";
                for (int x = XMIN; x < width; x++)
                {
                    if(Path[y][x] == OCCUPIED)
                        row.append(1,'+');
                    else if(Path[y][x] == SAFE)
                        row.append(1,' ');
                    else if(Path[y][x] == ISPATH)
                        row.append(1,'o');
                    else
                        row.append(1,' ');
                }
                newFile << row << std::endl;
            }
            newFile << std::endl <<std::endl;
            break;
        }
    }
    newFile.close();
    
}

bool ACML::isInPath(int x, int y)
{
      std::deque<Node*>::iterator iter = find(AStar_Path.begin(),AStar_Path.end(),&nodes[y*height + x]);
      if(iter != AStar_Path.end())
        return true;
      return false;
}

grid_Coordinate* ACML::t2g(true_Coordinate *t)
{
    int x = round((t->x - X_SHIFT)*XMAP_RATIO - 1);
    int y = round((t->y - Y_SHIFT)*YMAP_RATIO - 1);
    grid_Coordinate *g = new grid_Coordinate(); 
    g->x = x;
    g->y = y;
    if(x >= XMIN && x < width && y >= YMIN && y < height)
        return g;
    else
    {
        ROS_INFO("t2g error");
        printf("Error From:%f,%f\n",t->x,t->y);
        printf("Error Result:%d,%d\n",g->x,g->y);
        g->x = g->y = 0;
        return g;
    }
}

true_Coordinate* ACML::g2t(grid_Coordinate *g)
{
    float x = (float)(g->x+1)/XMAP_RATIO + X_SHIFT;
    float y = (float)(g->y+1)/YMAP_RATIO + Y_SHIFT;
    true_Coordinate *t = new true_Coordinate();
    t->x = x;
    t->y = y;
    return t;
}

int8_t** ACML::getMap2DArray()
{
    int8_t** Map = NEW2D(height,width,int8_t);
    for (size_t y = YMIN; y < height; y++)
    {
        for (size_t x = XMIN; x < width; x++)
        {
            Map[y][x] = my_map->data[width*y+x];    //Map[y][x]
            
        }

    }
    printf("Map done\n");
    return Map;
}

int8_t** ACML::getEMap(int8_t** Map)
{
    /*init eMap*/
    if(Map!=nullptr)
    {
        int8_t** eMap = NEW2D(height,width,int8_t);
        for (size_t y = YMIN; y < height; y++)
        {
            for (size_t x = XMIN; x < width; x++)
            {
                eMap[y][x] = EMPTY;
            }
        }

        /*enlarge obstacles to fit phsical car size*/ 
        for (size_t y = YMIN; y < height; y++)
        {
            for (size_t x = XMIN; x < width; x++)
            {
                int range = round(E_SIZE * 0.1 / my_map->info.resolution);

                /*if occupied then enlarge*/
                if(Map[y][x] == OCCUPIED)
                {
                    for(int y_bias = -range;y_bias < range+1;y_bias++)
                    {
                        for(int x_bias = -range;x_bias <range+1;x_bias++)
                        {
                            /*if in range*/
                            if(y+y_bias >= 0 && y+y_bias <= YMAX && x+x_bias >=0 && x+x_bias <= XMAX)
                            {
                                eMap[y+y_bias][x+x_bias] = OCCUPIED;
                            }
                        }
                    }
                }
                /*if eMap empty*/
                else if(eMap[y][x] == EMPTY)
                {
                    /*if nothing or unknown then copy*/
                    if(Map[y][x] == SAFE || Map[y][x] == UNKNOWN)
                    {
                        eMap[y][x] = Map[y][x];
                    }
                }  
            } 
        }
        printf("eMap done\n");
        return eMap;
    }
    ROS_WARN("Map is nullptr");
    return nullptr;
    
}

void ACML::initNodes()
{
    if (nodes == nullptr)
    {
        nodes = new Node[height*width];
        /*init Neighbors*/
        for(int y = YMIN; y < height; y++)
        {
            for(int x = XMIN;x < width; x++)
            {
                /*NSWE*/
                if(y>0)
                    nodes[y*width+x].vecNeighbors.push_back(&nodes[(y - 1)*width + (x + 0)]);
                if(y<YMAX)
                    nodes[y*width+x].vecNeighbors.push_back(&nodes[(y + 1)*width + (x + 0)]);
                if(x>0)
                    nodes[y*width+x].vecNeighbors.push_back(&nodes[(y + 0)*width + (x - 1)]);
                if(x<XMAX)
                    nodes[y*width+x].vecNeighbors.push_back(&nodes[(y + 0)*width + (x + 1)]);

                /*Diagonally*/
                if(x>0 && y>0)
                    nodes[y*width+x].vecNeighbors.push_back(&nodes[(y - 1)*width + (x - 1)]);
                if(y<YMAX && x>0)
                    nodes[y*width+x].vecNeighbors.push_back(&nodes[(y + 1)*width + (x - 1)]);
                if(x<XMAX && y>0)
                    nodes[y*width+x].vecNeighbors.push_back(&nodes[(y - 1)*width + (x + 1)]);
                if(x<XMAX && y<YMAX)
                    nodes[y*width+x].vecNeighbors.push_back(&nodes[(y + 1)*width + (x + 1)]);
            }
        }
    }
    /*init val*/
    for(int y = YMIN; y < height; y++)
    {
        for(int x = XMIN; x < width; x++)
        {
            Node *thisNode = &nodes[y*width+x];
            thisNode->self.x = x;
            thisNode->self.y = y;
            thisNode->parent = nullptr;
            thisNode->bVisited = false;
            thisNode->gCost = INFINITY;
            thisNode->fCost = INFINITY;
            if(eMap[y][x] == SAFE)
                thisNode->bObstacle = false;
            else
                thisNode->bObstacle = true;
        }
    }  
}

void ACML::Direction_Handler(Node* _current)
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

void ACML::solveAStar(Node *nodeStart, Node *nodeEnd)
{
    std::list<Node*> listNotTestedNodes;
    std::deque<Node*> path;
    if(nodeStart == nullptr)
    {
        /*init*/
        true_Coordinate t={
          .x = 0.0,
          .y = 0.0
        };
        grid_Coordinate* g = t2g(&t);
        nodeStart = &nodes[(g->y)*width+(g->x)];
    }

    assert(nodeEnd!=nullptr);

    /* A star */
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

    nodeCurrent = nodeEnd;
    int total = 0;
    int count = 0;
    while(nodeCurrent->parent != nullptr)
    {
        Direction_Handler(nodeCurrent);
        if(count > PATH_PUB_INTERVAL)
        {
            AStar_Path.push_front(nodeCurrent);
            count = 0;
        }
        nodeCurrent = nodeCurrent->parent;
        total++;
        count++;
    }
    AStar_Path.push_back(nodeEnd);
    printf("step count : %d\n",total);
}


int main(int argc, char** argv)
{
    ros::init(argc,argv,"ACML");
    ACML myACML;
    if(myACML.updateMapInfoOnce() && myACML.SubPubInit())
    {
        /* if map update ok then do sub and pub , if also ok enter main process */

        ROS_INFO("Init Done!");
        ros::Rate rate(LOOP_RATE);
        myACML.Enlargemap();
        while(ros::ok())
        {
            if(myACML.getNewGoal)
            {
                myACML.updateAStar();
                myACML.getNewGoal = false;
            }
            myACML.pubIfReached();
            ros::spinOnce();
            rate.sleep();  
        }
    }
    else
        ROS_WARN("Can't get anything from /map in waiting process or Sub/Pub failed");
    return 0;
}

