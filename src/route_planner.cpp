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

        // Mark the node as visited
        node->visited = true;

        // Add the node to the open list
        open_list.push_back(node);

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

std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {

    // Create path_found vector
    distance = 0.0f;
    std::vector<RouteModel::Node> path_found;

    RouteModel::Node* node = current_node;

    while (node) {

        // Get the distance from the node to its parent
        if (node->parent) distance += node->distance(*node->parent);

        // Push the node to the top of the vector
        path_found.push_back(*node);

        // Move up in the chain
        node = node->parent;
    }

    // Reverse vector to get correct path restructuring
    std::reverse(path_found.begin(), path_found.end());

    distance *= m_Model.MetricScale(); // Multiply the distance by the scale of the map to get meters.

    return path_found;
}

void RoutePlanner::AStarSearch() {

    // Add start node to the open list
    open_list.push_back(start_node);

    RouteModel::Node *current_node = nullptr;

    while (!open_list.empty()) {

        // sort the open_list and return node with lowest f
        current_node = NextNode();
        current_node->visited = true;

        // If goal reached, return reconstructed path
        if (current_node == end_node) {
            // Store final path in m_Model.path before method exits -> displayed on map title
            m_Model.path = ConstructFinalPath(current_node);
            return;
        }

        // Add neighbors of current node to the open_list
        AddNeighbors(current_node);

    }
}