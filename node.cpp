#include "astar_path_planner/node.h"

namespace astar_path_planner
{
std::ostream& operator<<(std::ostream& os, const Node& node)
{
  os << "    "
     << "ID: " << node.id << ", "
     << "Parent ID: " << node.parent_id << ", "
     << "Cost: " << node.cost << ", "
     << "Heuristic cost: " << node.heuristic_cost << std::endl;

  return os;
}
}