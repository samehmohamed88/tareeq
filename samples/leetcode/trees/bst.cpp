#include "binary_tree.hpp"

class Solution
{
public:
    TreeNode* searchBST(TreeNode* root, int val) {
        TreeNode* current = root;
        while (current != nullptr) {
            if (current->val == val) {
                return current;
            }
            if (current->val > val) {
                current = current->left;
            } else {
                current = current->right;
            }
        }
        return current;
    }
};

int main() {

    auto one = TreeNode(1);
    auto three = TreeNode(3);
    auto seven = TreeNode(7);

    TreeNode two = TreeNode(2, &one, &three);
    TreeNode root = TreeNode(4, &two, &seven);

    auto sol = Solution();
    auto node = sol.searchBST(&root, 2);

    return 0;
}