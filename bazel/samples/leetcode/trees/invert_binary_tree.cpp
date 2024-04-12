#include "binary_tree.hpp"

class Solution {
public:
    TreeNode* invertTree(TreeNode* root) {
        if (root == nullptr) {
            return nullptr;
        }
        TreeNode* tmp = root->right;
        root->right = root->left;
        root->left = tmp;

        invertTree(root->left);
        invertTree(root->right);
        return root;
    }
};
int main() {

    auto two = TreeNode(2);
//    auto one = TreeNode(1);
//    auto three = TreeNode(3);
//    auto six = TreeNode(6);
//    auto nine = TreeNode(9);
//
//    TreeNode seven = TreeNode(7, &six, &nine);
//    TreeNode two = TreeNode(2, &one, &three);
    TreeNode root = TreeNode(1, nullptr, &two);

    auto sol = Solution();
    TreeNode* node = sol.invertTree(&root);

    return 0;
}