#include "binary_tree.hpp"

#include <cmath>
#include <iostream>

class Solution {
public:
   bool isBalanced(TreeNode* root) {
       if (root == nullptr) {return true;}
       return std::abs(depth(root->left) - depth(root->right)) <= 1;
   }

   int depth(TreeNode* root) {
       if (root == nullptr) {return 0;}
       return 1 + std::max(depth(root->left), depth(root->right));
   }
};

int main() {
//    auto nine = TreeNode(9);
//    auto fifteen = TreeNode(1);
//    auto seven = TreeNode(7);
//    auto four = TreeNode(4);
//
//    TreeNode twenty = TreeNode(20, &fifteen, &seven);
//    TreeNode root = TreeNode(3, &nine, &twenty);

    TreeNode three = TreeNode(3);
    TreeNode two = TreeNode(2, nullptr, &three);
    TreeNode root = TreeNode(1, nullptr, &two);

    auto sol = Solution();
    std::cout << std::boolalpha << sol.isBalanced(&root) << std::endl;
    return 0;
}