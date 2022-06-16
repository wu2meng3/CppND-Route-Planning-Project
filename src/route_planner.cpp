#include "route_planner.h"
#include <algorithm>

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {
    // Convert inputs to percentage:
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;

    // Find the closest nodes to the starting and ending coordinates.
    start_node = &(m_Model.FindClosestNode(start_x, start_y));
    end_node = &(m_Model.FindClosestNode(end_x, end_y));
}


float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {
    return node->distance(*end_node);
}


void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {
    // FindNeighbors() takes care of visited nodes
    current_node->FindNeighbors();
    for (auto* node: current_node->neighbors) {
        node->parent = current_node;
        node->h_value = CalculateHValue(node);
        node->g_value = current_node->g_value + current_node->distance(*node);
        node->visited = true;
        open_list.push_back(node);   
    }
}


RouteModel::Node *RoutePlanner::NextNode() {
    // Sort in descending order
    std::sort(open_list.begin(), open_list.end(),
        [](const RouteModel::Node* a, const RouteModel::Node* b)
        {
            return (a->h_value + a->g_value) > (b->h_value + b->g_value);
        });
    auto* node_with_smallest_h_plus_g = open_list.back();
    open_list.pop_back();
    return node_with_smallest_h_plus_g;
}


std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
    // Create path_found vector
    distance = 0.0f;
    std::vector<RouteModel::Node> path_found;

    RouteModel::Node* node = current_node;
    constexpr float threshold = 1.0E-8;
    path_found.emplace_back(*node);

    while (node != start_node) {
        if (node->parent == nullptr) break;
        path_found.emplace_back(*(node->parent));
        distance += node->distance(*(node->parent));
        node = node->parent;
    }

    // Reverse the order of path_found such that the start node is the first
    // and the end node is the last.
    std::reverse(path_found.begin(), path_found.end());

    distance *= m_Model.MetricScale(); // Multiply the distance by the scale of the map to get meters.
    return path_found;

}


void RoutePlanner::AStarSearch() {
    RouteModel::Node *current_node = nullptr;

    constexpr float threshold = 1.0E-8;

    // Add start node to open_list
    current_node = start_node;
    current_node->parent = nullptr;
    current_node->h_value = CalculateHValue(current_node);
    current_node->g_value = 0;
    current_node->visited = true;
    open_list.push_back(current_node);  
    
    // while open_list is not empty, iterate open_list
    while (!open_list.empty()) {
        current_node = NextNode();
        // Reach end node, set path
        if (current_node->distance(*end_node) < threshold) {
            m_Model.path = ConstructFinalPath(end_node);
            break;
        }
        AddNeighbors(current_node);
    }
}