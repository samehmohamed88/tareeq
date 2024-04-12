#include "binary_tree.hpp"

class Solution
{
public:
    int diameterOfBinaryTree(TreeNode* root) {

    }
};

int main() {

    auto three = TreeNode(3);
    auto five = TreeNode(5);
    auto four = TreeNode(4);

    TreeNode two = TreeNode(2, &four, &five);
    TreeNode root = TreeNode(1, &two, &three);

    auto sol = Solution();
    auto diameter = sol.diameterOfBinaryTree(&root);

    return 0;
}