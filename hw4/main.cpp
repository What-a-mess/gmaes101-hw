#include <chrono>
#include <iostream>
#include <string>
#include <opencv2/opencv.hpp>

std::vector<cv::Point2f> control_points;
int count = 4;
int times[700][700];

void mouse_handler(int event, int x, int y, int flags, void *userdata) 
{
    if (event == cv::EVENT_LBUTTONDOWN && control_points.size() < count) 
    {
        std::cout << "Left button of the mouse is clicked - position (" << x << ", "
        << y << ")" << '\n';
        control_points.emplace_back(x, y);
    }     
}

void naive_bezier(const std::vector<cv::Point2f> &points, cv::Mat &window) 
{
    auto &p_0 = points[0];
    auto &p_1 = points[1];
    auto &p_2 = points[2];
    auto &p_3 = points[3];

    for (double t = 0.0; t <= 1.0; t += 0.001) 
    {
        auto point = std::pow(1 - t, 3) * p_0 + 3 * t * std::pow(1 - t, 2) * p_1 +
                 3 * std::pow(t, 2) * (1 - t) * p_2 + std::pow(t, 3) * p_3;

        window.at<cv::Vec3b>(point.y, point.x)[2] = 255;
    }
}

cv::Point2f recursive_bezier(const std::vector<cv::Point2f> &control_points, float t) 
{
    // TODO: Implement de Casteljau's algorithm
    if (control_points.size() == 2) {
        return (1-t) * control_points[0] + t * control_points[1];
    }

    std::vector<cv::Point2f> next_level_points;
    for (int i = 0; i < control_points.size() - 1; i++) {
        next_level_points.push_back((1-t) * control_points[i] + t * control_points[i+1]);
    }

    return recursive_bezier(next_level_points, t);

}

void bezier(const std::vector<cv::Point2f> &control_points, cv::Mat &window) 
{
    // TODO: Iterate through all t = 0 to t = 1 with small steps, and call de Casteljau's 
    // recursive Bezier algorithm.
    for (double t = 0.0; t <= 1.0; t += 0.001) 
    {
        auto point = recursive_bezier(control_points, t);
        int x_int = point.x;
        int y_int = point.y;

        float x_ratio = point.x - x_int;
        float y_ratio = point.y - y_int;

        int time;
        
        // time = times[x_int][y_int];
        // if (time) {
        //     window.at<cv::Vec3b>(point.y, point.x)[2] *= (time - 1) / (time * 1.0f);
        //     window.at<cv::Vec3b>(point.y, point.x)[2] += 255 * (1-x_ratio) * (1-y_ratio) * 1.0f / time;
        // } else {
        //     window.at<cv::Vec3b>(point.y, point.x)[2] = 255 * (1-x_ratio) * (1-y_ratio);
        // }
        // times[x_int][y_int]++;

        // time = times[x_int + 1][y_int];
        // if (time) {
        //     window.at<cv::Vec3b>(point.y, point.x + 1)[2] *= (time - 1) / (time * 1.0f);
        //     window.at<cv::Vec3b>(point.y, point.x + 1)[2] += 255 * x_ratio * (1-y_ratio) * 1.0f / time;
        // } else {
        //     window.at<cv::Vec3b>(point.y, point.x + 1)[2] = 255 * x_ratio * (1-y_ratio);
        // }
        // times[x_int + 1][y_int]++;

        // time = times[x_int][y_int + 1];
        // if (time) {
        //     window.at<cv::Vec3b>(point.y + 1, point.x)[2] *= (time - 1) / (time * 1.0f);
        //     window.at<cv::Vec3b>(point.y + 1, point.x)[2] += 255 * (1-x_ratio) * y_ratio * 1.0f / time;
        // } else {
        //     window.at<cv::Vec3b>(point.y + 1, point.x)[2] = 255 * (1-x_ratio) * y_ratio;
        // }
        // times[x_int][y_int + 1]++;

        // time = times[x_int + 1][y_int + 1];
        // if (time) {
        //     window.at<cv::Vec3b>(point.y + 1, point.x + 1)[2] *= (time - 1) / (time * 1.0f);
        //     window.at<cv::Vec3b>(point.y + 1, point.x + 1)[2] += 255 * x_ratio * y_ratio * 1.0f / time;
        // } else {
        //     window.at<cv::Vec3b>(point.y + 1, point.x + 1)[2] = 255 * x_ratio * y_ratio;
        // }
        // times[x_int + 1][y_int + 1]++;

        if (window.at<cv::Vec3b>(point.y, point.x)[2] < 255 * (1-x_ratio) * (1-y_ratio))
            window.at<cv::Vec3b>(point.y, point.x)[2] = 255 * (1-x_ratio) * (1-y_ratio);
        // window.at<cv::Vec3b>(point.y, point.x)[2] = 255;
        if (window.at<cv::Vec3b>(point.y, point.x + 1)[2] < 255 * x_ratio * (1-y_ratio))
            window.at<cv::Vec3b>(point.y, point.x + 1)[2] = 255 * x_ratio * (1-y_ratio);
        if (window.at<cv::Vec3b>(point.y + 1, point.x)[2] < 255 * (1-x_ratio) * y_ratio)
            window.at<cv::Vec3b>(point.y + 1, point.x)[2] = 255 * (1-x_ratio) * y_ratio;
        if (window.at<cv::Vec3b>(point.y + 1, point.x + 1)[2] < 255 * x_ratio * y_ratio)
            window.at<cv::Vec3b>(point.y + 1, point.x + 1)[2] = 255 * x_ratio * y_ratio;
    }
}

int main(int argc, char *argv[]) 
{
    if (argc > 0) {
        count = std::stoi(argv[1]);
    }

    cv::Mat window = cv::Mat(700, 700, CV_8UC3, cv::Scalar(0));
    cv::cvtColor(window, window, cv::COLOR_BGR2RGB);
    cv::namedWindow("Bezier Curve", cv::WINDOW_AUTOSIZE);

    cv::setMouseCallback("Bezier Curve", mouse_handler, nullptr);

    int key = -1;
    while (key != 27) 
    {
        for (auto &point : control_points) 
        {
            cv::circle(window, point, 3, {255, 255, 255}, 3);
        }

        if (control_points.size() == count) 
        {
            // naive_bezier(control_points, window);
            bezier(control_points, window);

            cv::imshow("Bezier Curve", window);
            cv::imwrite("my_bezier_curve.png", window);
            key = cv::waitKey(0);
            std::system("pause");

            return 0;
        }

        cv::imshow("Bezier Curve", window);
        key = cv::waitKey(20);
    }

return 0;
}
