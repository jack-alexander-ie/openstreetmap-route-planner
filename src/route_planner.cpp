#include "route_planner.h"
#include <algorithm>

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {
    // Convert inputs to percentage:
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;

    start_node = &m_Model.FindClosestNode(start_x, start_y);
    end_node = &m_Model.FindClosestNode(end_x, end_y);
}

float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {
    return node->distance(*end_node);
}

void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {

    // Populates neighbour vector with nodes
    current_node->FindNeighbors();

    // Set parent, g & h values for each neighbouring node
    for (RouteModel::Node *node : current_node->neighbors) {
        node->parent = current_node;
        node->g_value = current_node->g_value + node->distance(*current_node);
        node->h_value = CalculateHValue(node);

        // Add the node to the open list
        open_list.push_back(node);

        // Mark the node as visited
        node->visited = true;
    }
}

RouteModel::Node *RoutePlanner::NextNode() {

    // Sorting comparison solution found: https://knowledge.udacity.com/questions/44755
    sort(open_list.begin(), open_list.end(), [](const auto &node1, const auto &node2) {
        return node1->h_value + node1->g_value < node2->h_value + node2->g_value;
    });

    // Create a pointer to the node with the lowest f value
    RouteModel::Node* lowestValue = open_list[0];

    // Remove lowest node from open list
    open_list.erase(open_list.begin());

    // Return the pointer to the lowest value
    return lowestValue;
}


// TODO 6:
//  Complete the ConstructFinalPath method to return the final path found from your A* search.
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

    // TODO: Implement your solution here.

    distance *= m_Model.MetricScale(); // Multiply the distance by the scale of the map to get meters.
    return path_found;

}


// TODO 7:
//  Write the A* Search algorithm here.
// Tips:
// - Use the AddNeighbors method to add all of the neighbors of the current node to the open_list.
// - Use the NextNode() method to sort the open_list and return the next node.
// - When the search has reached the end_node, use the ConstructFinalPath method to return the final path that was found.
// - Store the final path in the m_Model.path attribute before the method exits. This path will then be displayed on the map tile.

void RoutePlanner::AStarSearch() {
    RouteModel::Node *current_node = nullptr;

    // TODO: Implement your solution here.

}