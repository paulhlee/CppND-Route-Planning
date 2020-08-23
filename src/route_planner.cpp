#include "route_planner.h"
#include <algorithm>

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {
    // Convert inputs to percentage:
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;
	
    // TODO 2: Use the m_Model.FindClosestNode method to find the closest nodes to the starting and ending coordinates.
    // Store the nodes you find in the RoutePlanner's start_node and end_node attributes.
  	start_node = &model.FindClosestNode(start_x, start_y);
	end_node = &model.FindClosestNode(end_x, end_y);
}


// TODO 3: Implement the CalculateHValue method.
// Tips:
// - You can use the distance to the end_node for the h value.
// - Node objects have a distance method to determine the distance to another node.

float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {
  
	return node->distance(*end_node);
}


// TODO 4: Complete the AddNeighbors method to expand the current node by adding all unvisited neighbors to the open list.
// Tips:
// - Use the FindNeighbors() method of the current_node to populate current_node.neighbors vector with all the neighbors.
// - For each node in current_node.neighbors, set the parent, the h_value, the g_value. 
// - Use CalculateHValue below to implement the h-Value calculation.
// - For each node in current_node.neighbors, add the neighbor to open_list and set the node's visited attribute to true.

void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {
		current_node->FindNeighbors();
  		for(RouteModel::Node *nNode: current_node->neighbors ){
          nNode->parent = current_node;
          nNode->h_value = this->CalculateHValue(nNode);
          float distanceP = nNode->distance(*current_node);
          nNode->g_value = current_node->g_value +distanceP;
          this->open_list.push_back(nNode);
          nNode->visited = true;
          
        }
 		current_node -> visited = true;

}


bool RoutePlanner::comparison (RouteModel::Node *nodex ,RouteModel::Node *nodey ) { 
  float xVal = nodex->h_value + nodex->g_value;
  float yVal = nodey->h_value + nodey->g_value;
  return (xVal<yVal); 
}
// TODO 5: Complete the NextNode method to sort the open list and return the next node.
// Tips:
// - Sort the open_list according to the sum of the h value and g value.
// - Create a pointer to the node in the list with the lowest sum.
// - Remove that node from the open_list.
// - Return the pointer.

RouteModel::Node *RoutePlanner::NextNode() {

  std::sort(open_list.begin(),open_list.end(), RoutePlanner::comparison);
  RouteModel::Node *lowest = open_list[0];
  open_list.erase(open_list.begin());
  return lowest;

}


// TODO 6: Complete the ConstructFinalPath method to return the final path found from your A* search.
// Tips:
// - This method should take the current (final) node as an argument and iteratively follow the 
//   chain of parents of nodes until the starting node is found.
// - For each node in the chain, add the distance from the node to its parent to the distance variable.
// - The returned vector should be in the correct order: the start node should be the first element
//   of the vector, the end node should be the last element.

std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
    // Create path_found vector
    distance = 0.0f;
    std::vector<RouteModel::Node> path_found;
	RouteModel::Node current = *current_node;
  	path_found.push_back(current);
  	distance = distance+ current.g_value;
    // TODO: Implement your solution here.
//   	std::cout << current.g_value * m_Model.MetricScale()<<"\n";
  	distance = current.g_value;
	while(current.parent){
      RouteModel::Node *parent = current.parent;
      current = *parent;
      path_found.push_back(current);
//       distance = distance + current.g_value;
//       std::cout << current.g_value * m_Model.MetricScale()<<"\n";
    }
  	
    distance *= m_Model.MetricScale(); // Multiply the distance by the scale of the map to get meters.
    std::reverse(path_found.begin(), path_found.end());
//   	std::cout << distance <<"\n";
    return path_found;

}

bool RoutePlanner::compareNode(RouteModel::Node *nodex ,RouteModel::Node *nodey ){


  return (nodex->x == nodey->x) && (nodex->y == nodey->y);
}
// TODO 7: Write the A* Search algorithm here.
// Tips:
// - Use the AddNeighbors method to add all of the neighbors of the current node to the open_list.
// - Use the NextNode() method to sort the open_list and return the next node.
// - When the search has reached the end_node, use the ConstructFinalPath method to return the final path that was found.
// - Store the final path in the m_Model.path attribute before the method exits. This path will then be displayed on the map tile.

void RoutePlanner::AStarSearch() {
    RouteModel::Node *current_node = nullptr;
	current_node = start_node;
  	start_node->visited = true;
  	this->AddNeighbors(current_node);
  	bool notfound;
  	notfound = true;
  	if (compareNode(current_node, end_node)){
      std::vector<RouteModel::Node> finalPath = this->ConstructFinalPath(current_node);
      m_Model.path = finalPath;
      notfound = false;
    }
  	while((!open_list.empty())&&(notfound)){
      current_node = this->NextNode();
      if (compareNode(current_node,end_node)){
        std::vector<RouteModel::Node> finalPath = this->ConstructFinalPath(current_node);
//         std::cout << "out"<<"\n";
        notfound = false;
        m_Model.path = finalPath;
        break;
      }
      else{
//         std::cout << "loop"<<"\n";
        this->AddNeighbors(current_node);        
      }
 
    }
  	
  	

  

}