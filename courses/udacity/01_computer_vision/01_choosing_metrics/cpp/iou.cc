#include "utils.h"

#include <nlohmann/json.hpp>

#include <unistd.h>
#include <limits.h>

void debug() {
    char cwd[PATH_MAX];
    if (getcwd(cwd, sizeof(cwd)) != nullptr) {
        std::cout << "Current working dir: " << cwd << std::endl;
    } else {
        std::cerr << "getcwd() error" << std::endl;
    }
}

int main() {
    auto json_data = get_data();
    auto gt_bboxes = get_bounding_boxes(
            "segment-1231623110026745648_480_000_500_000_with_camera_labels_38.png",
            json_data.first);
    auto pre_bboxes = get_bounding_boxes("segment-1231623110026745648_480_000_500_000_with_camera_labels_38.png",
                                         json_data.second);
    auto gt_matrix = getBoundingBoxesAsMatrix(gt_bboxes);
    auto pred_matrix = getBoundingBoxesAsMatrix(pre_bboxes);

    return 0;
}
