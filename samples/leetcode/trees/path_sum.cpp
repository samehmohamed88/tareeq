
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
    TreeNode* hasPathSum(TreeNode* root, int val) {
        
    }
};

int main() {

    auto one = TreeNode(1);
    auto two = TreeNode(2);
    auto seven = TreeNode(7);
    auto thirteen = TreeNode(13);

    TreeNode eleven = TreeNode(11, &seven, &two);
    TreeNode four = TreeNode(4, nullptr, &one);

    TreeNode eight = TreeNode(8, &thirteen, &four);

    TreeNode four_again = TreeNode(4, &eleven, nullptr);

    TreeNode root = TreeNode(5, &four_again, &eight);

    auto sol = Solution();
    auto node = sol.searchBST(&root, 2);

    return 0;
}