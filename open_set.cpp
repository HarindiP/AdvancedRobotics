#include "astar_path_planner/open_set.h"
#include <limits>
#include <ros/ros.h>

namespace astar_path_planner
{
void OpenSet::push(const Node& n)
{
  nodes_.push_back(n);
}

Node OpenSet::pop(double heuristic_cost_weight)
{
  int index = 0; //saves the id
 // double best_combined_cost = nodes_[0].cost + ( nodes_[0].heuristic_cost * heuristic_cost_weight   ; //saves the cost

 double lowest_combined_cost = std::numeric_limits<double>::max();

  int temp = 0; 
    

  // Find the best node in "nodes_"
  // Save the index into "index" and it will be removed from "nodes_" and returned
  // You need to compare combined costs: the cost of the node + (heuristic cost * weight)
  // Use "heuristic_cost_weight" to calculate the combined cost so it can be modified later

  // YOUR CODE HERE
     
  // for loop that goes throght all values in the vector and try to get the node with the lowest heuristic cost
 
  for (int i = 0 ; i < ( sizeof(nodes_)) ; i++)
  {
	double combined = nodes_[i].cost + ( nodes_[i].heuristic_cost * heuristic_cost_weight);

	if ( combined < lowest_combined_cost )
	{
		lowest_combined_cost = combined; 
		index = i ; 
	}

  }

  
   
  // YOU DON'T NEED TO MODIFY ANYTHING AFTER THIS LINE

  // Copy the node
  Node n = nodes_[index];

  // Overwrite the node with the node at the end of the vector
  nodes_[index] = nodes_.back();

  // Delete the node at the end of the vector
  nodes_.pop_back();

  // Return the copy
  return n;
}

bool OpenSet::contains(int id)
{
  // Returns true if the node is in nodes_
  for (const auto& n : nodes_)
  {
    if (n.id == id)
    {
      return true;
    }
  }

  return false;
}

void OpenSet::update(const Node& n)
{
  // Find node "n" in "nodes_"
  // If the cost of node "n" is less than the cost of the node already in the open set, replace it

  // YOUR CODE HERE

  for (auto& a : nodes_)
  { 
    if ( a.id == n.id)
    {
      if (a.cost > n.cost)
      {
	//this line might be wrong
	a = n ; 

      }

    }

  }

  

}

bool OpenSet::empty()
{
  return nodes_.empty();
}

const std::vector<Node>& OpenSet::getNodes()
{
  return nodes_;
}

std::ostream& operator<<(std::ostream& os, const OpenSet& open_set)
{
  os << "\n\nOpen set:" << std::endl;

  for (const auto& n : open_set.nodes_)
  {
    os << n;
  }

  return os;
}

}  // namespace astar_path_planner
