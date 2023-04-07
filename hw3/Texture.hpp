//
// Created by LEI XU on 4/27/19.
//

#ifndef RASTERIZER_TEXTURE_H
#define RASTERIZER_TEXTURE_H
#include "global.hpp"
#include <eigen3/Eigen/Eigen>
#include <opencv2/opencv.hpp>
class Texture{
private:
    cv::Mat image_data;

public:
    Texture(const std::string& name)
    {
        image_data = cv::imread(name);
        cv::cvtColor(image_data, image_data, cv::COLOR_RGB2BGR);
        width = image_data.cols;
        height = image_data.rows;
    }

    int width, height;

    Eigen::Vector3f getColor(float u, float v)
    {
        auto u_img = u * width;
        auto v_img = (1 - v) * height;
        // if (u < 0 || v < 0) std::cout << u_img << ' ' << v_img << '\n';
        auto color = image_data.at<cv::Vec3b>(v_img, u_img);
        return Eigen::Vector3f(color[0], color[1], color[2]);
    }

    Eigen::Vector3f getColorBilinear(float u, float v) 
    {
        if (u < 0) u = 0;
        if (u > 1) u = 1;
        if (v < 0) u = 0;
        if (v > 1) v = 1;

        float u_img = u * width;
        float v_img = (1 - v) * height;
        int int_u = round(u_img);
        int int_v = round(v_img);

        int u_min = std::max(0, (int)floor(u_img));
        int u_max = std::min(width, (int)ceil(u_img));
        int v_min = std::max(0, (int)floor(v_img));
        int v_max = std::min(height, (int)ceil(v_img));

        auto color_lefttop = image_data.at<cv::Vec3b>((int_v - 1 + height) % height, (int_u - 1 + width) % width);
        auto color_leftbottom = image_data.at<cv::Vec3b>(int_v, (int_u - 1 + width) % width);
        auto color_righttop = image_data.at<cv::Vec3b>((int_v - 1 + height) % height, int_u);
        auto color_rightbottom = image_data.at<cv::Vec3b>(int_v, int_u);

        // auto color_leftbottom = image_data.at<cv::Vec3b>(v_max, u_min);
        // auto color_lefttop = image_data.at<cv::Vec3b>(v_min, u_min);
        // auto color_rightbottom = image_data.at<cv::Vec3b>(v_max, u_max);
        // auto color_righttop = image_data.at<cv::Vec3b>(v_min, u_max);

        float dU = 0.5 + (u_img - int_u);
        float dV = 0.5 + (v_img - int_v);

        // float dU = (u_img - u_min) / (u_max - u_min);
        // float dV = (v_img - v_min) / (v_max - v_min);

        auto color_bottom = color_leftbottom * (1-dU) + color_rightbottom * (dU);
        auto color_top = color_lefttop * (1-dU) + color_righttop * (dU);
        auto color = color_bottom * dV + color_top * (1-dV);

        return Eigen::Vector3f(color[0], color[1], color[2]);
    }

};
#endif //RASTERIZER_TEXTURE_H
