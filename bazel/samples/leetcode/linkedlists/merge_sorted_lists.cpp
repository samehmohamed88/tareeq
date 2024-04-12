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

#include <iostream>


struct ListNode {
    int val;
    ListNode *next;
    ListNode() : val(0), next(nullptr) {}
    ListNode(int x) : val(x), next(nullptr) {}
    ListNode(int x, ListNode *next) : val(x), next(next) {}
};

class Solution {
public:
    ListNode* mergeTwoLists(ListNode* list1, ListNode* list2) {
        if (list1 == NULL && list2 == NULL) {
            return NULL;
        }
        if (list1 == NULL) {
            return list2;
        }
        if (list2 == NULL) {
            return list1;
        }

        ListNode* dummy = new ListNode();
        ListNode *curr = dummy;
        while (list1 != NULL && list2 != NULL) {
            if (list1->val <= list2->val) {
                curr->next = list1;
                list1 = list1->next;
            } else {
                curr->next = list2;
                list2 = list2->next;
            }
            curr = curr->next;
        }

        if (list1 == NULL) {
            curr->next = list2;
        } else {
            curr->next = list1;
        }

        return dummy->next;
    }
};

int main() {

    /**
    Input: list1 = [1,2,4], list2 = [1,3,4]
    Output: [1,1,2,3,4,4]
     */

    auto sol = Solution();

    auto One_four = ListNode(4);
    ListNode One_two = ListNode(2, &One_four);
    ListNode One_one = ListNode(1, &One_two);

//    ListNode One_one = ListNode(5, nullptr);

    auto Two_four = ListNode(4);
    ListNode Two_three = ListNode(3, &Two_four);
    ListNode Two_one = ListNode(1, &Two_three);

    auto res = sol.mergeTwoLists(&One_one, &Two_one);



    return 0;
}