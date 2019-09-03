#include <iostream>
#include <cassert>
#include <chrono>

#include "rrt.hh"


int test_node_length(){
    Node n1(std::vector<double> {10.0, 10.0, 10.0, 10.0});

    assert(n1.length() == 20.0);

    return 0;
}

int test_random_node(){
    return 0;
}


int test_equal_node(){
    Node n1(std::vector<double> {10.0, 10.0, 10.0, 10.0});
    Node n2(std::vector<double> {10.0, 10.0, 10.0, 10.0});

    auto is_equal = n1.is_equal(n2);
    assert(is_equal == true);

    Node n3(std::vector<double> {11.0, 10.0, 10.0, 10.0});
    is_equal = n1.is_equal(n3);
    assert(is_equal == false);

    return 0;
}

int test_rrt_tree(){

    Node start(std::vector<double> {0.0});
    Node goal(std::vector<double> {1.0});

    RRT_Tree tree (start, goal, std::vector<std::vector<double>> {{0.0, 2.0}}, 100);

    for (int i=0; i<100; i++)
    {
        const auto rand_node = tree.get_random_config();
        const auto values = rand_node.get_values();
        // std::cout << values.at(0) << ", ";
        assert(values.size()==1);
        assert(values.at(0) >= 0.0);
        assert(values.at(0) <= 2.0);
    }

    return 0;
}

int test_tree_nearest(){
    Node start(std::vector<double> {0.0});
    Node goal(std::vector<double> {1.0});

    RRT_Tree tree (start, goal, std::vector<std::vector<double>> {{0.0, 2.0}}, 100);

    for (double step = 0.0; step < 1.0; step+=0.1){
        tree.add_node(Node(std::vector<double> {step}));
    }

    const auto indx1 = tree.get_nearest_vertex_index(
                                Node(std::vector<double> {1.1}));

    assert(indx1 == 1);

    const auto indx2 = tree.get_nearest_vertex_index(
                                Node(std::vector<double> {0.11}));
    const auto nearest = tree.get_node_from_indx(indx2);

    assert(abs(nearest.distance(goal)) <= 0.1);

    return 0;
}

int test_same_node(){
    Node start(std::vector<double> {0.0});
    Node goal(std::vector<double> {1.0});

    RRT_Tree tree (start, goal, std::vector<std::vector<double>> {{0.0, 2.0}}, 100);
    
    assert(tree.is_near_at_rand(start, start));
    assert(!tree.is_near_at_rand(start, goal));

    return 0;
}


int main()
{
    auto start_time = std::chrono::high_resolution_clock::now();
    int result = 0;

    result = test_node_length();
    std::cout << "Result of test_node_length: " << result << std::endl;

    result = test_random_node();
    std::cout << "Result of test_random_node: " << result << std::endl;

    result = test_equal_node();
    std::cout << "Result of test_equal_node: " << result << std::endl;

    result = test_rrt_tree();
    std::cout << "Result of test_rrt_tree: " << result << std::endl;

    result = test_tree_nearest();
    std::cout << "Result of test_tree_nearest: " << result << std::endl;

    result = test_same_node();
    std::cout << "Result of test_same_node: " << result << std::endl;

    auto end_time = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed = (end_time - start_time);
    std::cout << "Time to run tests: " << elapsed.count() << " s\n";

    return 0;
}
