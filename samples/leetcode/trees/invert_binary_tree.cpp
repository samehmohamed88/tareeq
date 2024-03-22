#include <iostream>

struct TreeNode
{
    int val;
    TreeNode* left;
    TreeNode* right;
    TreeNode()
        : val(0)
        , left(nullptr)
        , right(nullptr)
    {}
    TreeNode(int x)
        : val(x)
        , left(nullptr)
        , right(nullptr)
    {}
    TreeNode(int x, TreeNode* left, TreeNode* right)
        : val(x)
        , left(left)
        , right(right)
    {}
};

class Solution {
public:
    TreeNode* invertTree(TreeNode* root) {
        if (root == nullptr) {
            return nullptr;
        }
        if (root->left == nullptr && root->right == nullptr) {
            return nullptr;
        }
        if (invertTree(root->left) == nullptr) {
            TreeNode* tmp = root->right;
            root->right = root->left;
            root->left = tmp;
        }
        if (invertTree(root->right) == nullptr) {
            TreeNode* tmp = root->right;
            root->right = root->left;
            root->left = tmp;
        }
        return root;
    }

};
int main() {

    auto one = TreeNode(1);
    auto three = TreeNode(3);
//    auto six = TreeNode(6);
//    auto nine = TreeNode(9);
//
//    TreeNode seven = TreeNode(7, &six, &nine);
//    TreeNode two = TreeNode(2, &one, &three);
    TreeNode root = TreeNode(2, &one, &three);

    auto sol = Solution();
    TreeNode* node = sol.invertTree(&root);

    return 0;
}