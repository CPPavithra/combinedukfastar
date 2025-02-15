#include <librealsense2/rs.hpp>
#include <iostream>
#include <vector>
#include <rerun.hpp>
#include <Eigen/Dense>
#include <thread>
#include <mutex>
#include <chrono>
#include <unordered_map>
#include <functional>
#include <Eigen/Core>
//for downsampling and filtering
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <cstdlib>
#include <rerun/demo_utils.hpp>
#include <unordered_set>
#include <sstream>
#include "../include/rerun.h"
#include "common.h"
#include "UKF.h"

//chrono is for time
using namespace rerun;
using namespace rs2;

//GLOBALLY
/*Gridmap gridmap;
grid_resolution = 0.001f; //because the distance is in mm and we have to convert it o metr 
batch_threshold = 2;*/

void create_gridmap(Gridmap& gridmap,const vector<Vector3f>& point_vectors, const Pose& roverpose,float grid_resolution, float height,float proxfactor)//declare only in rerun.h
{
        float boundary_threshold = 0.01f;
	if (roverpose.position(0)<gridmap.min_x+boundary_threshold) 
	{
           gridmap.min_x-=(grid_resolution*50.0f);
        }
        if (roverpose.position(0)>gridmap.max_x-boundary_threshold) 
	{
           gridmap.max_x+=(grid_resolution*50.0f);
        }
        if (roverpose.position(1)<gridmap.min_y+boundary_threshold) 
	{
           gridmap.min_y-=(grid_resolution*50.0f);
        }
        if (roverpose.position(1)>gridmap.max_y-boundary_threshold)
       	{
           gridmap.max_y+=(grid_resolution*50.0f);
        }

       unordered_map<pair<int,int>,CellCost, pair_hash>updated_occupancy_grid=gridmap.occupancy_grid;
       int proxradius=3;
      	 
   	float rover_x=roverpose.position.x();
        float rover_y=roverpose.position.y();

      	// to verify if it is valid
	//cout<<"Rover position (real-world): ("<<rover_x<<", "<<rover_y<<")"<<endl;
       for (const auto& point : point_vectors)
       {
          int grid_x = static_cast<int>((point.z() / grid_resolution)/1000); // Map to grid cell
          int grid_y = static_cast<int>((point.x() / grid_resolution)/1000);
          float height_at_point = point.y(); // Use z for height


          float cellsize=0.5f; //metre
          //cout << "Mapped grid position: (" << grid_x << ", " <<grid_y << ")" <<endl;
          // cout<<"Height at ("<<tolog_x<<" , "<<tolog_y<<") ->"<<roverpose.position.z()<<"\n"<<endl;
       
          //to calculate cost
       	  float cost=0.0f;
       	  if(height_at_point>height)
       	  {
           	cost=10.0f; //very high=cant go cost is from range 0 to 10
       	  }
       	  else if(height_at_point>(height/2) &&height_at_point<=(height))
       	  {
           	cost=5.0f;
       	  }
       	  else if(height_at_point>(height/4) && height_at_point<=(height/2))
       	  {
           	cost=1.0f;
       	  }

	  std::cout << "Mapped grid position: (" << grid_x  << ", " << grid_y << "), COST ->" <<cost<<"\n"<< std::endl;
	 
    
     // USE BIT MASK ENCODING TO CHECK IF THE NODE IS VISITED OR NOT, I have used visited and proxvisited boolean visited and proxvisited with the cost proxcost.
         pair<int, int> current = {grid_x,grid_y};
         //std::cout << "Before updating: (" << current.first << ", " << current.second << ") -> Cost: " << cost << std::endl;
	 CellCost& cell=updated_occupancy_grid[current];

	float proxcostupdate=cell.proxcost;
	bool proxvupdate=cell.proxvisited;
		 
        if (!cell.visited && !cell.proxvisited) {  //CHECK HERE
            //update the cost AND mark them as visited to avoid re-iteration of that cell
            cell.cost += cost;  //adding new cost to the existing cost (the existing cost might be proximity cost= proxcost)
            cell.visited = true; //mark that cell as visited to avoid reiteration
            cell.proxvisited = proxvupdate;
            if (cell.cost == 0.0f) {
               updated_occupancy_grid.erase(current);
            }
       }
       else {
        //if not found in the occupancy grid then visit that node and then update it
         if(cost>0.0f) {
           updated_occupancy_grid[current] = CellCost(cost,proxcostupdate,true,false);
         }  // CellCost(float c = 0.0f, pc=0.0f, bool v = false, bool p = false) : cost(c),proxcost,(pc), visited(v),proxvisited(p)
       }
       cout<< "After Updating: ("<< current.first<< ", "<< current.second<< ") -> Cost: "<< cost<<endl;

//////////////////////////////////////////////////
//NEIGHBOURING COST PADDING
   /*float prox=1; //edit as needed
    for (float dx=-prox; dx<=prox;++dx) {
        for (float dy=-prox; dy <=prox ;++dy) {
            if (dx == 0 && dy == 0) continue;  // Skip the current cell

            int neighbor_x = grid_x + dx;
            int neighbor_y = grid_y + dy;
            pair<int, int> neighbor = {neighbor_x, neighbor_y};
           //add new neighbor if not already in the grid
	   if(updated_occupancy_grid[{grid_x,grid_y}].visited)
	   {
           if (updated_occupancy_grid.find(neighbor)==updated_occupancy_grid.end()) {
               updated_occupancy_grid[neighbor]=CellCost{0.0f, 0.0f, false, false};  //set cost as default
               std::cout<< "Added new neighbor: ("<< neighbor.first<< ", "<< neighbor.second<< ")" << std::endl;
	   }

        CellCost& neighbor_cell=updated_occupancy_grid[neighbor];
       
       
        //calculate proximity cost
        float dist = sqrt(dx *dx + dy*dy);  /*grid_resolution*/;//euclidean distance between them. 
     /*  float proxcost = (proxfactor*2.0f)/(0.1f+dist);

        //update proximity cost only if not already updated
       if (neighbor_cell.proxvisited==false) {
            neighbor_cell.proxcost = proxcost;
            neighbor_cell.cost += neighbor_cell.proxcost; //add proximity cost
            neighbor_cell.proxvisited = true;            //mark it as visited for proximity
        }
	}
	}
   }*/
}

// Initialize the boundaries to extreme values
       int min_x = INT_MAX;
       int max_x = INT_MIN;
       int min_y = INT_MAX;
       int max_y = INT_MIN;

for (const auto& cell : updated_occupancy_grid) {
    int xindice = cell.first.first;  // Extract grid x index
    int yindice = cell.first.second; // Extract grid y index

    if (xindice < min_x) min_x =xindice;
    if (xindice > max_x) max_x =xindice;
    if (yindice < min_y) min_y =yindice;
    if (yindice > max_y) max_y =yindice;
}

gridmap.min_x=min_x;
gridmap.min_y=min_y;
gridmap.max_x=max_x;
gridmap.max_y=max_y;

    std::cout << "Updated occupancy grid size: " << updated_occupancy_grid.size() << std::endl;
    for (const auto& [key, value] : updated_occupancy_grid) 
    {
	std::cout << "Grid: (" << key.first << ", " << key.second << ") -> " << value.cost << std::endl;
    }
   gridmap.occupancy_grid=updated_occupancy_grid;
   std::cout << "After assignment: Occupancy grid size: " << gridmap.occupancy_grid.size() << std::endl;
   //updated_occupancy_grid[current]=CLOSED; //after everything

}
/////////////////////////



//color based on cost
components::Color get_color_for_cost(const CellCost& cell) 
{
  if (cell.proxvisited && cell.cost > 0.0f && cell.cost<1.0f) {
          return components::Color{
              static_cast<uint8_t>(0.0f * 255),
              static_cast<uint8_t>(0.0f * 255),
              static_cast<uint8_t>(1.0f * 255)
          };  // BLUE = Proxvisited and cost > 0
 }
   else	if (cell.cost >= 10.0f)
 {
           return components::Color{
              static_cast<uint8_t>(0.0f * 255), 
              static_cast<uint8_t>(0.0f * 255), 
              static_cast<uint8_t>(0.0f * 255)
            };
}  // BLACK=FULLY OCCUPIED
 else if (cell.cost >= 5.0f) 
 {
           return components::Color{
             static_cast<uint8_t>(1.0f * 255), 
             static_cast<uint8_t>(0.0f * 255), 
             static_cast<uint8_t>(0.0f * 255)
           };
 }// RED=MILD }
 else if (cell.cost >= 1.0f)
{
          return components::Color{
             static_cast<uint8_t>(1.0f * 255), 
             static_cast<uint8_t>(0.8f * 255), 
             static_cast<uint8_t>(0.4f * 255)
          };
}	// ORANGE=CAN GO } 
else
{
    return components::Color{
          static_cast<uint8_t>(0.6f * 255), 
          static_cast<uint8_t>(1.0f * 255), 
          static_cast<uint8_t>(0.6f * 255)
    };
}	// LIGHT GREEN } 
}
     

void draw_gridmap(const Gridmap& gridmap,const vector<Vector3f>& point_vectors, const Pose& roverpose, float grid_resolution, rerun::RecordingStream& rec)
{
    float min_x=(roverpose.position.x()-5.0f)/grid_resolution;
    float max_x=(roverpose.position.x()+5.0f)/grid_resolution;
    float min_y=(roverpose.position.y()-5.0f)/grid_resolution;
    float max_y=(roverpose.position.y()+5.0)/grid_resolution;
    float scale_factor = 1000.0f;  // I have put 1000 so that it is in mm
    std::cout << "Occupancy grid size: " << gridmap.occupancy_grid.size() << std::endl;
    if (gridmap.occupancy_grid.empty())
    {
        std::cout << "Error: Occupancy grid is empty!" << std::endl;
    }
    else
    {
        std::cout << "Occupancy grid has data!" << std::endl;
    }
    std::vector<rerun::Color> colors;
    std::vector<rerun::Position3D> roverposition;
    std::ostringstream table_data;
    //table_data << "Position (x, y) | Color (r, g, b) | Cost\n";
    for (const auto& entry : gridmap.occupancy_grid)
   {
    const auto& [coord, value] = entry;
    float grid_x = coord.first;
    float grid_y = coord.second;

     rerun::Color color = get_color_for_cost(value);
    
     std::vector<rerun::Position3D> points = {rerun::Position3D{grid_x, grid_y, 0.0f}};

      
     colors.push_back(color);
     std::string tag = "gridcell_(" + std::to_string(grid_x) + "," + std::to_string(grid_y) + ")_"+ std::to_string(value.cost);
     rec.log(tag, rerun::Points3D(points).with_colors({color}).with_radii({0.5f}));
}
colors.clear();
}


Eigen::Vector3f convert_to_eigen_vector(const rs2_vector& rs2_vec) {
	return Eigen::Vector3f(rs2_vec.x, rs2_vec.y, rs2_vec.z);
}//helper function to convert rs2 to eigen vector3f


//function to update the rover's pose using IMU data
/*void update_rover_pose(struct Pose& pose, const Vector3f& accel_data, const Vector3f& gyro_data, float delta_time) {
    //gravity vector in the world frame
    Vector3f gravity(0.0f, 0.0f, -9.81f);
    //convert acceleration from the sensor frame to the world frame
    Vector3f accel_world=pose.orientation*accel_data;

    //subtract gravity from the acceleration
    accel_world=accel_world-gravity;
    //v=u+at
    pose.velocity=pose.velocity+accel_world*delta_time;

    //s=ut+0.5at^2
    Vector3f delta_position=(pose.velocity*delta_time)+(0.5f*accel_world*delta_time*delta_time);
    pose.position=pose.position+delta_position;

    //convert position from millimeters to meters if required
    pose.position=pose.position/1000.0f;

    //compute angular velocity from gyroscope data
    Vector3f angular_velocity=gyro_data*delta_time;

    //update orientation using a small-angle quaternion
    Quaternionf delta_q=Quaternionf(AngleAxisf(angular_velocity.norm(),angular_velocity.normalized()));
    pose.orientation=delta_q*pose.orientation;
    pose.orientation.normalize(); //normalize quaternion to prevent drift

    cout<< "Position: "<<pose.position.transpose() <<endl;
    cout<< "Velocity: " <<pose.velocity.transpose() <<endl;
}*/
////////////////////////////
const size_t BUFFER_SIZE = 10; // Define the size of the circular buffer
std::vector<Vector3d> circular_buffer(BUFFER_SIZE, Vector3d::Zero()); // Initialize with zeros
size_t buffer_index = 0; // Start index for the buffer

// Function to update the circular buffer with new state data
void updateBuffer(const Vector3d& new_state) {
    circular_buffer[buffer_index] = new_state;
    buffer_index = (buffer_index + 1) % BUFFER_SIZE;
}

// Function to retrieve the most recent state from the buffer
Vector3d getLatestState() {
    size_t latest_index = (buffer_index + BUFFER_SIZE - 1) % BUFFER_SIZE;
    return circular_buffer[latest_index];
}
////////////////////////////////
void update_rover_pose(Pose& pose) {
    Vector3d state = locate();
    updateBuffer(state);
    Vector3d latest_state = getLatestState(); // Get the latest position and
                                              // yaw

    // Assign x, y, yaw to the rover's pose
    pose.position(0) = latest_state(0);//x
    pose.position(1) = latest_state(1);//y
    pose.position(2) = latest_state(2);//yaw

    // Compute new orientation from yaw
    float yaw = latest_state(2); // Assuming yaw is in radians
    pose.orientation = Quaternionf(AngleAxisf(yaw, Vector3f::UnitZ()));
    pose.orientation.normalize();

    // Print updated pose
    cout << "Updated Position - x: " << pose.position(0)
         << ", y: " << pose.position(1)
         << ", yaw: " << pose.position(2) << endl;

    cout << "Updated Orientation (Quaternion): " << pose.orientation.coeffs().transpose() << endl;
}
    //update orientation using a small-angle quaternion
//    Quaternionf delta_q=Quaternionf(AngleAxisf(angular_velocity.norm(),angular_velocity.normalized()));
//    pose.orientation=delta_q*pose.orientation;
//    pose.orientation.normalize(); //normalize quaternion to prevent drift
//convert the realsense points to pcl point
   pcl::PointCloud<pcl::PointXYZ>::Ptr convert_to_pcl(const std::vector<Eigen::Vector3f>& point_vectors) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    for (const auto& point : point_vectors) {
        cloud->points.emplace_back(point.x(), point.y(), point.z());
    }
    cloud->width = cloud->points.size();
    cloud->height = 1; // Unorganized cloud
    return cloud;
}

