#include <utility>

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

class Solution
{
public:

    TreeNode* insertIntoBST(TreeNode* root, int val) {
        if (root == nullptr) {
            return new TreeNode(val);
        }
        if (root->val > val) {
            root->left = insertIntoBST(root->left, val);
        } else {
            root->right = insertIntoBST(root->right, val);
        }
        return root;
    }
};

int main() {

    auto one = TreeNode(1);
    auto three = TreeNode(3);
    auto seven = TreeNode(7);

    TreeNode two = TreeNode(2, &one, &three);
    TreeNode root = TreeNode(4, &two, &seven);

    auto sol = Solution();
    auto node = sol.insertIntoBST(&root, 5);


    return 0;
}