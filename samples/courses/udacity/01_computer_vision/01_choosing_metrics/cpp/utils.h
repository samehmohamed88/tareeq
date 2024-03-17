#include <cnpy.h>
#include <Eigen/Dense>
#include <nlohmann/json.hpp>

#include <iostream>
#include <fstream>
#include <utility>
#include <string>
#include <vector>
#include <array>
#include <memory>
#include <algorithm>

class BoundingBox {
public:
    BoundingBox(int x1, int y1, int x2, int y2)
        : x1_{x1}
        , y1_{y1}
        , x2_{x2}
        , y2_{y2}
    {};

    BoundingBox(std::array<int, 4> boundingBox)
        : x1_{boundingBox[0]}
        , y1_{boundingBox[1]}
        , x2_{boundingBox[2]}
        , y2_{boundingBox[3]}
    {};

    int getX1() {return x1_;};
    int getX2() {return x2_;};

    int getY1() {return y1_;};
    int getY2() {return y2_;};

    int getArea() {
        return (x2_ - x1_) * (y2_ - y1_);
    };
private:
    int x1_, y1_, x2_, y2_;
};

const nlohmann::json& get_bounding_boxes(const std::string& filename, const nlohmann::json& data) {
    for (const auto& item : data) {
        if (item.contains("filename") && item["filename"] == filename) {
            return item["boxes"];
        }
    }
    return {};
}

std::pair<nlohmann::json, nlohmann::json> get_data() {
    // Relative paths from the current working directory to the JSON files
    std::string gt_filePath = "01_computer_vision/01_choosing_metrics/data/ground_truth.json";
    std::string pred_filePath = "01_computer_vision/01_choosing_metrics/data/predictions.json";

    nlohmann::json gt_json, pred_json;

    std::ifstream gt_file(gt_filePath);
    if (gt_file.is_open()) {
        gt_json = nlohmann::json::parse(gt_file);
    } else {
        std::cerr << "Could not open " << gt_filePath << std::endl;
    }
    gt_file.close();

    std::ifstream  pred_file(pred_filePath);
    if (pred_file.is_open()) {
        pred_json = nlohmann::json::parse(pred_file);
    } else {
        std::cerr << "Could not open  " << pred_filePath << std::endl;
    }
    pred_file.close();

    return {gt_json, pred_json};
}

Eigen::MatrixXi getBoundingBoxesAsMatrix(const nlohmann::json& boxesJson) {
    // Create an Eigen matrix of the appropriate size
    Eigen::MatrixXi matrix(boxesJson.size(), 4);

    // Fill the Eigen matrix with the data from JSON
    for (Eigen::Index i = 0; i < boxesJson.size(); ++i) {
        for (Eigen::Index j = 0; j < 4; ++j) {
            matrix(i, j) = boxesJson[i][j];
        }
    }

    return matrix;
}