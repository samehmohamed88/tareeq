/**
* Definition for singly-linked list.
* struct ListNode {
*     int val;
*     ListNode *next;
*     ListNode() : val(0), next(nullptr) {}
*     ListNode(int x) : val(x), next(nullptr) {}
*     ListNode(int x, ListNode *next) : val(x), next(next) {}
* };
*/


struct ListNode {
    int val;
    ListNode *next;
    ListNode() : val(0), next(nullptr) {}
    ListNode(int x) : val(x), next(nullptr) {}
    ListNode(int x, ListNode *next) : val(x), next(next) {}
};

class Solution {
public:
   ListNode* reverseList(ListNode* head) {
//        ListNode* two = head->next;
//        ListNode* three = two->next;
//        three->next = two;
//        two->next = three;
//        head->next = nullptr;
        // head -> 2 -> 3 ->
        // step a:

        
        return head;
   }
};

int main() {

    return 0;
}