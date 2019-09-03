// Author: Joseph Lowman
#pragma once

#include <cmath>
#include <iostream>
#include <random>
#include <vector>
#include <map>


class Node
{
public:
    Node (std::vector<double> values){
        values_ = values;
    }

    double length() const{
        double l = 0.0;
        for (const auto v: values_){
            l += (v*v);
        }

        l = std::sqrt(l);

        return l;
    }

    std::vector<double> get_values() const{
        return values_;
    }

    bool is_equal(const Node & other) const{
        return values_ == other.get_values();
    }

    double distance(const Node & other) const{
        double dist=0;

        for (auto indx=0; indx<values_.size(); indx++){
            const auto delta = (values_.at(indx) - other.get_values().at(indx));
            dist += delta*delta;
        }
    }

private:
    std::vector<double> values_;
};


struct TreeConfig{
    std::vector<std::vector<double>> limits;
    int num_iterations;
};

class RRT_Tree
{
public:

    RRT_Tree(Node start, Node goal, 
                std::vector<std::vector<double>> limits,
                int num_iterations):

                start_(start.get_values()),
                goal_(goal.get_values()),
                step_size_ (0.01),
                limits_ (limits),
                num_iterations_ (num_iterations),
                engine_(seeder_())
    {
        add_node(start_);
        add_node(goal_);

        finished_ = false;

    }


    void build_graph(int num_iterations){
        for (auto i = 0; i < num_iterations_; i++){
            const auto rand_node = get_random_config();
            const auto near_node_indx = get_nearest_vertex_index(rand_node);
            const auto near_node = get_node_from_indx(near_node_indx);

            while (!is_near_at_rand(near_node, rand_node)){
                const auto step_node = get_new_vertex(near_node, rand_node);
                
                add_node(step_node);

                const int step_node_indx = nodes_.size()-1;

                add_edge(near_node_indx, step_node_indx);
            }
        }

        return;
    }

    Node get_random_config(){

        std::vector<double> rand_config_values;

        for (auto joint_limit: limits_){
            const auto min = joint_limit.at(0);
            const auto max = joint_limit.at(1);

            std::uniform_int_distribution<int> dist(0,
                            static_cast<int>((max-min)/step_size_));
            const int rand_int = dist(engine_);

            const double rand_val = min + rand_int*step_size_;
            rand_config_values.push_back(rand_val);
        }

        return Node(rand_config_values);
    }

    int get_nearest_vertex_index(const Node & rand_node) const{
        double shortest_dist = 100000000;
        int nearest_node_indx = 0;

        int indx = 0;
        for (auto node: nodes_){
            auto dist = rand_node.distance(node);
            if (dist < shortest_dist){
                shortest_dist = dist;
                nearest_node_indx = indx;
            }

            indx++;
        }

        return nearest_node_indx;
    }


    bool is_near_at_rand(const Node & near_node, const Node & rand_node) const{
        return near_node.distance(rand_node) <= step_size_;
    }


    Node get_new_vertex(const Node & near_node, const Node & rand_node) const{
        const auto dist = near_node.distance(rand_node);

        std::vector<double> new_values;

        for (int i=0; i<near_node.get_values().size(); i++){
            const auto delta = rand_node.get_values().at(i) - near_node.get_values().at(i);
            const auto val = near_node.get_values().at(i) + step_size_/dist*delta;
        }

        return Node(new_values);
    }


    void add_node(const Node & node){
        nodes_.push_back(node);
    }


    void add_edge(const int & near_node_indx, const int & new_node_indx){

        edges_indx_[new_node_indx] = std::vector<int>{near_node_indx};
        edges_indx_[near_node_indx].push_back(new_node_indx);

        return;
    }

    Node get_node_from_indx(const int & node_indx) const{
        // TODO: ensure node_inx < nodes_.size();
        return nodes_.at(node_indx);
    }

private:
    std::vector<Node> nodes_;
    std::map<int, std::vector<int>> edges_indx_;

    Node start_;
    Node goal_;

    double step_size_;
    std::vector<std::vector<double>> limits_;
    int num_iterations_;

    bool finished_;

    std::random_device seeder_;
    std::mt19937 engine_;
};
