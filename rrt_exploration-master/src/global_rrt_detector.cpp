#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include <iostream>
#include <string>
#include <vector>
#include "stdint.h"
#include "functions.h"
#include "mtrand.h"


#include "nav_msgs/OccupancyGrid.h"
#include "geometry_msgs/PointStamped.h"
#include "std_msgs/Header.h"
#include "nav_msgs/MapMetaData.h"
#include "geometry_msgs/Point.h"
#include "visualization_msgs/Marker.h"
#include <tf/transform_listener.h>

#include <unistd.h>


// global variables
nav_msgs::OccupancyGrid mapData;
geometry_msgs::PointStamped clickedpoint;
geometry_msgs::PointStamped exploration_goal;
visualization_msgs::Marker points,line;
float xdim,ydim,resolution,Xstartx,Xstarty,init_map_x,init_map_y;

rdm r; // for genrating random numbers



//Subscribers callback functions---------------------------------------
void mapCallBack(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
mapData=*msg;
ROS_INFO("global: Get the map");
}


 
void rvizCallBack(const geometry_msgs::PointStamped::ConstPtr& msg)   /* zhangjiadong 话题/clicked_point */
{ 
// 前四点是四个定义要探索的正方形区域，最后一个点是树的起点。在发表了关于这个话题的五点后，RRT将开始检测边界
geometry_msgs::Point p;  
p.x=msg->point.x;
p.y=msg->point.y;
p.z=msg->point.z;

points.points.push_back(p);
ROS_INFO("Get the [%d]th clicked_point -- callback", points.points.size());
}




int main(int argc, char **argv)
{

  unsigned long init[4] = {0x123, 0x234, 0x345, 0x456}, length = 7;
  MTRand_int32 irand(init, length); // 32-bit int generator
// this is an example of initializing by an array
// you may use MTRand(seed) with any 32bit integer
// as a seed for a simpler initialization
  MTRand drand; // double in [0, 1) generator, already init

// generate the same numbers as in the original C test program
  ros::init(argc, argv, "global_rrt_frontier_detector");
  ros::NodeHandle nh;
  
  // fetching all parameters
  float eta,init_map_x,init_map_y,range;
  std::string map_topic,base_frame_topic;
  
  std::string ns;
  ns=ros::this_node::getName();

  ros::param::param<float>("/eta", eta, 0.5);
  ros::param::param<std::string>("/map_topic", map_topic, "/map"); 
//---------------------------------------------------------------
ros::Subscriber sub= nh.subscribe(map_topic, 100 ,mapCallBack);	
ros::Subscriber rviz_sub= nh.subscribe("/clicked_point", 100 ,rvizCallBack);	

ros::Publisher targetspub = nh.advertise<geometry_msgs::PointStamped>("/detected_points", 10);  // “/detected_points”是rrt-tree上， 存在未知区域的point。将作为潜在的探测目标。
ros::Publisher pub = nh.advertise<visualization_msgs::Marker>("/local_detected_shapes", 10);  //[my]   zhangjiadong ： 将/clicked_point转化为用于可视化的/local_detected_shapes中的五个点，决定了探测的范围、

ros::Rate rate(100); 
 
 
// TODO: [zhangjiadong] 这什么鬼啊？？？　　为什么mapData明明有了数据，但却data.size为０
// wait until map is received, when a map is received, mapData.header.seq will not be < 1  
// while (mapData.header.seq<1 or mapData.data.size()<1)  {  
// 	ros::spinOnce();  ros::Duration(0.1).sleep();
// 	ROS_INFO("global: wait until map is received, %f,  %f", mapData.header.seq, mapData.data.size());
// 	std::cout<<"map_topic:"<<map_topic<<std::endl;
// }


//visualizations  points and lines..
points.header.frame_id=mapData.header.frame_id;
line.header.frame_id=mapData.header.frame_id;
points.header.stamp=ros::Time(0);
line.header.stamp=ros::Time(0);
	
points.ns=line.ns = "markers";
points.id = 0;
line.id =1;


points.type = points.POINTS;
line.type=line.LINE_LIST;

//Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
points.action =points.ADD;
line.action = line.ADD;
points.pose.orientation.w =1.0;
line.pose.orientation.w = 1.0;
line.scale.x =  0.03;
line.scale.y= 0.03;
points.scale.x=0.3; 
points.scale.y=0.3; 

line.color.r =9.0/255.0;   //线（global rrt）是蓝色的
line.color.g= 91.0/255.0;
line.color.b =236.0/255.0;
points.color.r = 255.0/255.0;  //点（click point）是红色的
points.color.g = 0.0/255.0;
points.color.b = 0.0/255.0;
points.color.a=1.0;
line.color.a = 1.0;
points.lifetime = ros::Duration();
line.lifetime = ros::Duration();

geometry_msgs::Point p;  


while(points.points.size()<5)     /* zhangjiadong 话题/clicked_point 。RRT中设置至少添加5个点，才会运行*/
{
	ros::spinOnce();
	pub.publish(points) ;  //说明，接收到小于五个point时，才发送出去。第六个就无效了。
	ROS_INFO("Get the [%d]th clicked_point", points.points.size());
	// std::cout<<"Get the "<<points.points.size()<<"th clicked_point"<<std::endl;
}



// 以下代码用来计算RRT的初始点
std::vector<float> temp1;
temp1.push_back(points.points[0].x);
temp1.push_back(points.points[0].y);
	
std::vector<float> temp2; 
temp2.push_back(points.points[2].x);
temp2.push_back(points.points[0].y);


init_map_x=Norm(temp1,temp2);
temp1.clear();		temp2.clear();

temp1.push_back(points.points[0].x);
temp1.push_back(points.points[0].y);

temp2.push_back(points.points[0].x);
temp2.push_back(points.points[2].y);

init_map_y=Norm(temp1,temp2);
temp1.clear();		temp2.clear();

Xstartx=(points.points[0].x+points.points[2].x)*.5;
Xstarty=(points.points[0].y+points.points[2].y)*.5;





geometry_msgs::Point trans;
trans=points.points[4];
std::vector< std::vector<float>  > V; 
std::vector<float> xnew; 
xnew.push_back( trans.x);xnew.push_back( trans.y);  
V.push_back(xnew);

points.points.clear();
pub.publish(points) ;







std::vector<float> frontiers;
int i=0;
float xr,yr;
//最重要的循环过程： 随机采样点,最近的顶点,新的子节点
std::vector<float> x_rand,x_nearest,x_new;


// Main loop
while (ros::ok()){

		// Sample free 参数控制用于检测边界点的RRT的增长率，单位为米。分别生成随机点，在树上找到与xr最近的节点xnearest，以及再根据eta产生一个新的随机点。
		x_rand.clear();
		xr=(drand()*init_map_x)-(init_map_x*0.5)+Xstartx;
		yr=(drand()*init_map_y)-(init_map_y*0.5)+Xstarty;


		x_rand.push_back( xr ); x_rand.push_back( yr );


		// Nearest
		x_nearest=Nearest(V,x_rand);

		// Steer

		x_new=Steer(x_nearest,x_rand,eta);


		// 接下来，计算新产生的随机点与最临近点之间的障碍物： ObstacleFree    1:free     -1:unkown (frontier region)      0:obstacle
		char   checking=ObstacleFree(x_nearest,x_new,mapData);

			if (checking==-1){
				// 如果当前点与最临近点之间未建图，即未知的情况下，通过发布exploration_goal目标点，探索位置区域。
					exploration_goal.header.stamp=ros::Time(0);
					exploration_goal.header.frame_id=mapData.header.frame_id;
					exploration_goal.point.x=x_new[0];
					exploration_goal.point.y=x_new[1];
					exploration_goal.point.z=0.0;
					p.x=x_new[0]; 
					p.y=x_new[1]; 
					p.z=0.0;
					points.points.push_back(p);
					pub.publish(points) ;
					targetspub.publish(exploration_goal);  //这咋办？？
					points.points.clear();
					
					}
				
				//   如果未检测到障碍物的话，则直接连成线。
			else if (checking==1){
				V.push_back(x_new);
				
				p.x=x_new[0]; 
				p.y=x_new[1]; 
				p.z=0.0;
				line.points.push_back(p);
				p.x=x_nearest[0]; 
				p.y=x_nearest[1]; 
				p.z=0.0;
				line.points.push_back(p);

					}



		pub.publish(line);  //line的颜色


		

		ros::spinOnce();
		rate.sleep();
	}
  return 0;
}
