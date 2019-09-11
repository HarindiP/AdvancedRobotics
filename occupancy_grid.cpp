#include "astar_path_planner/occupancy_grid.h"

namespace astar_path_planner
{
OccupancyGrid::OccupancyGrid(const nav_msgs::OccupancyGrid& map, const double inflation_radius)
{
  // Copy the occupancy grid message
  map_ = map;

  // Access occupancy grid message data with an image
  map_image_ = cv::Mat(map.info.height, map.info.width, CV_8U, &map_.data.front());

  // Dilate the image
  int element_diameter =
      2 * static_cast<int>(std::round(inflation_radius / map.info.resolution)) + 1;  // element_diameter is always odd

  int offset = (element_diameter - 1) / 2;  // Centre of the element

  cv::Mat element =
      cv::getStructuringElement(cv::MORPH_RECT, cv::Size(element_diameter, element_diameter), cv::Size(offset, offset));

  cv::dilate(map_image_, map_image_, element);

  // Map geometry for particle filter
  map_x_min_ = map_.info.origin.position.x;
  map_x_max_ = map_.info.width * map_.info.resolution + map_.info.origin.position.x;

  map_y_min_ = map_.info.origin.position.y;
  map_y_max_ = map_.info.height * map_.info.resolution + map_.info.origin.position.y;
}

bool OccupancyGrid::isOutOfBounds(GridPosition grid_position)
{
  return grid_position.x < 0 || grid_position.x > map_image_.cols ||  //
         grid_position.y < 0 || grid_position.y > map_image_.rows;
}

bool OccupancyGrid::isOutOfBounds(WorldPosition world_position)
{
  return world_position.x < map_x_min_ || world_position.x > map_x_max_ ||  //
         world_position.y < map_y_min_ || world_position.y > map_y_max_;
}

bool OccupancyGrid::isOccupied(int id)
{
  return map_.data[id] != 0;
}

bool OccupancyGrid::isOccupied(GridPosition grid_position)
{
  return isOccupied(getCellId(grid_position));
}

bool OccupancyGrid::isOccupied(WorldPosition world_position)
{
  return isOccupied(getGridPosition(world_position));
}

GridPosition OccupancyGrid::getGridPosition(int id)
{
  return { id % static_cast<int>(map_.info.width), id / static_cast<int>(map_.info.width) };
}

GridPosition OccupancyGrid::getGridPosition(WorldPosition world_position)
{
  GridPosition grid_position{};

  grid_position.x =
      static_cast<int>(std::floor((world_position.x - map_.info.origin.position.x) / map_.info.resolution));

  grid_position.y =
      static_cast<int>(std::floor((world_position.y - map_.info.origin.position.y) / map_.info.resolution));

  return grid_position;
}

WorldPosition OccupancyGrid::getWorldPosition(GridPosition grid_position)
{
  WorldPosition world_position{};

  world_position.x = static_cast<double>(grid_position.x) * map_.info.resolution + map_.info.origin.position.x +
                     (map_.info.resolution / 2.);
  world_position.y = static_cast<double>(grid_position.y) * map_.info.resolution + map_.info.origin.position.y +
                     (map_.info.resolution / 2.);

  return world_position;
}

WorldPosition OccupancyGrid::getWorldPosition(int id)
{
  return getWorldPosition(getGridPosition(id));
}

int OccupancyGrid::getCellId(GridPosition grid_position)
{
  return grid_position.y * static_cast<int>(map_.info.width) + grid_position.x;
}

Cell OccupancyGrid::getCell(int id)
{
  Cell cell{};

  cell.id = id;
  cell.occupied = isOccupied(id);
  cell.grid_position = getGridPosition(id);
  cell.world_position = getWorldPosition(cell.grid_position);

  return cell;
}

Cell OccupancyGrid::getCell(GridPosition grid_position)
{
  return getCell(getCellId(grid_position));
}

Cell OccupancyGrid::getCell(WorldPosition world_position)
{
  return getCell(getGridPosition(world_position));
}

nav_msgs::OccupancyGrid OccupancyGrid::getOccupancyGridMsg()
{
  return map_;
}

std::vector<AdjacentCell> OccupancyGrid::getAdjacentCells(int id, bool diagonal_movement)
{
  // Return the unoccupied cells adjacent to "id"

  // Grid position of the given cell, use this to get adjacent cell grid positions
  GridPosition grid_position = getGridPosition(id);

  // Fill this with adjacent cells
  std::vector<AdjacentCell> adjacent_cells{};

  bool check_diagonal = diagonal_movement ; 

  // Use "isOutOfBounds" and "isOccupied" to check if the adjacent cells are out of bounds or occupied
  // You can use world postion to get grid position
  // The "AdjacentCell" structure has three fields: "id", "cost", and "world_position of center of the cell"
  // Use "getCellId" and "getWorldPosition" to get a cell ID and world position from a grid position
  // "cost" is the cost of moving from the parent to the adjacent cell in metres
  // "map_.info.resolution" is the distance between cells
  // Only return diagonal cells if "diagonal_movement" is true
  // Keep in mind that the distance between diagonal cells is larger than horizontal/vertical cells

  // YOUR CODE HERE
  
  //remember all predesssor of the neighboor cells is id so neighbour's parent id : id
 
  //for when diagonal is set to a no
  if (diagonal_movement == false)
  {
    //temp variable to hold neighboor cell info
    GridPosition neighbour;
    
    //counter to see how many cells have been observed
    int count  = 0;
  
    //iterates through left right
    for (int i = -1; i  < 2 ; i++)
    {
      for (int j = -1; j  < 2 ; j++)
	{
	  
		
	// code only conducted for left up down right (in that order)
	if (((i = -1)  &&  ( j = 0))  ||  ((i = 0)  &&  (j = 1))  ||  ((i =0)  &&  (j = -1))  ||  ((i = 1)  &&  (j = 0)))
	{
	    neighbour.x = grid_position.x + i ; 
	    neighbour.y = grid_position.y + i ; 
	
	   // i know i dont need the middle node the node that started it al but i dont know how to avoid it
	   //play with temp is the new grid pos and then update grid pos
	  
	   if( ((!isOutOfBounds(neighbour)) && (!isOccupied(neighbour))) == true)
	   {
	     //the cell is not out of bounds AND is not occupied do you thang gurl
	      //contruct the adjacent cell and return the appropro shits
      
	      int id = getCellId(neighbour) ;
	      double cost = sqrt ((pow(map_.info.resolution , 2)) + (pow(map_.info.resolution , 2)));
	      WorldPosition wp = getWorldPosition(neighbour);
	  

	      //push into the vector adjacent cell
	      adjacent_cells[count].id = id ;
	      adjacent_cells[count].cost = cost ;  
	      adjacent_cells[count].world_position = wp;

	      //increment counter
	      count++;
	  }
	}
      }
    }
  }

  //for when diagonal is set to a yes
  else if (diagonal_movement == true)
  {
    //temp variable to hold neighboor cell info : how to ini
    GridPosition neighbour;

    //counter to see how many cells have been observed
    int count  = 0;

    //create a for loop that iterates througgh the neighbours
    for (int i = -1 ; i < 2 ; i ++)
    {
      for (int j = -1 ; j< 2 ; j++)

	neighbour.x = grid_position.x + i ; 
	neighbour.y = grid_position.y + i ; 
	
        // i know i dont need the middle node the node that started it al but i dont know how to avoid it
	//play with temp is the new grid pos and then update grid pos
	//
	if( ((!isOutOfBounds(neighbour)) && (!isOccupied(neighbour))) == true)
	{
	  //the cell is not out of bounds AND is not occupied do you thang gurl
	  //contruct the adjacent cell and return the appropro shits
      
	  int id = getCellId(neighbour) ;
	  double cost = sqrt ((pow(map_.info.resolution , 2)) + (pow(map_.info.resolution , 2)));
	  WorldPosition wp = getWorldPosition(neighbour);
	  

	  //push into the vector adjacent cell
	  adjacent_cells[count].id = id ;
	  adjacent_cells[count].cost = cost ;  
	  adjacent_cells[count].world_position = wp;

	  //increment counter
	  count++;
	}
    }
  }

  

  return adjacent_cells;
}

}  // namespace astar_path_planner
