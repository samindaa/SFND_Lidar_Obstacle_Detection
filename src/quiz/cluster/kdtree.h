/* \author Aaron Brown */
// Quiz on implementing kd tree

#include "../../render/render.h"

// Structure to represent node of kd tree
struct Node {
  std::vector<float> point;
  int id;
  Node *left;
  Node *right;

  Node(std::vector<float> arr, int setId)
      : point(arr), id(setId), left(NULL), right(NULL) {}
};

struct KdTree {
  Node *root;

  KdTree() : root(NULL) {}

  void insert(std::vector<float> point, int id) {
    // TODO: Fill in this function to insert a new point into the tree
    // the function should create a new node and place correctly with in the
    // root
    insert_helper(root, point, id, 0);
  }

  // return a list of point ids in the tree that are within distance of target
  std::vector<int> search(std::vector<float> target, float distanceTol) {
    std::vector<int> ids;
    search_helper(ids, target, root, distanceTol, 0);
    return ids;
  }

private:
  void insert_helper(Node *&node, std::vector<float> &point, int id, size_t d) {
    if (node == nullptr) {
      node = new Node(point, id);
    } else if (point[d % point.size()] < node->point[d % point.size()]) {
      insert_helper(node->left, point, id, ++d);
    } else {
      insert_helper(node->right, point, id, ++d);
    }
  }

  void search_helper(std::vector<int> &ids, const std::vector<float> &target,
                     const Node *node, float distance_tol, size_t d) {
    if (node == nullptr) {
      return;
    }

    auto box_check = [](const float &q, const float &t, const float &dist) {
      return ((-dist <= (q - t)) && ((q - t) <= dist));
    };

    auto cir_check = [](const std::vector<float> &q,
                        const std::vector<float> &t, const float &dist) {
      float norm2 = 0.0F;
      for (size_t i = 0; i < q.size(); ++i) {
        norm2 += std::pow(q[i] - t[i], 2);
      }
      return norm2 <= dist * dist;
    };

    bool box_check_status = true;
    for (size_t i = 0; i < node->point.size(); ++i) {
      if (!box_check(node->point[i], target[i], distance_tol)) {
        box_check_status = false;
        break;
      }
    }
    if (box_check_status && cir_check(node->point, target, distance_tol)) {
      ids.emplace_back(node->id);
    }

    if (target[d % target.size()] - distance_tol <
        node->point[d % target.size()]) {
      search_helper(ids, target, node->left, distance_tol, ++d);
    }
    if (target[d % target.size()] + distance_tol >
        node->point[d % target.size()]) {
      search_helper(ids, target, node->right, distance_tol, ++d);
    }
  }
};
