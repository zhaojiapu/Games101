#include <chrono>
#include <iostream>
#include <opencv2/opencv.hpp>

std::vector<cv::Point2f> control_points;

void mouse_handler(int event, int x, int y, int flags, void *userdata) 
{
    if (event == cv::EVENT_LBUTTONDOWN && control_points.size() < 4)
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
    // 该函数使用一个控制点序列和一个浮点数 t 作为输入，实现 de Casteljau 算法来返回 Bézier 曲线上对应点的坐标。
    //
    // De Casteljau 算法说明如下:
    // 1. 考虑一个 p0, p1, ... pn 为控制点序列的 Bézier 曲线。首先，将相邻的点连接起来以形成线段。
    // 2. 用 t : (1 − t) 的比例细分每个线段，并找到该分割点。
    // 3. 得到的分割点作为新的控制点序列，新序列的长度会减少一。
    // 4. 如果序列只包含一个点，则返回该点并终止。否则，使用新的控制点序列并转到步骤 1。
    // 使用 [0,1] 中的多个不同的 t 来执行上述算法，你就能得到相应的 Bézier 曲线。

    // TODO: Implement de Casteljau's algorithm

    if(control_points.size() == 1) return control_points[0];

    std::vector<cv::Point2f> new_control_point = {};

    for (int i=0; i<control_points.size()-1; ++i) {
        auto & p0 = control_points[i];
        auto & p1 = control_points[i+1];
        auto p = p0 + t * (p1 - p0);
        new_control_point.emplace_back(p);
    }

    return recursive_bezier(new_control_point, t);

}

void bezier(const std::vector<cv::Point2f> &control_points, cv::Mat &window) 
{
    // 该函数实现绘制 Bézier 曲线的功能。
    // 它使用一个控制点序列和一个 OpenCV::Mat 对象作为输入，没有返回值。
    // 它会使 t 在 0 到 1 的范围内进行迭代，并在每次迭代中使 t 增加一个微小值。
    // 对于每个需要计算的 t，将调用另一个函数 recursive_bezier，然后该函数将返回在 Bézier 曲线上 t 处的点。
    // 最后，将返回的点绘制在 OpenCV::Mat 对象上。

    // TODO: Iterate through all t = 0 to t = 1 with small steps, and call de Casteljau's 
    // recursive Bezier algorithm.

    for (double t = 0.0; t <= 1.0; t += 0.001) {
        cv::Point2f point = recursive_bezier(control_points, t);
        window.at<cv::Vec3b>(point.y, point.x)[1] = 255;
    }

}

int main() 
{
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

        if (control_points.size() == 4)
        {
            naive_bezier(control_points, window);
            bezier(control_points, window);

            cv::imshow("Bezier Curve", window);
            cv::imwrite("my_bezier_curve.png", window);
            key = cv::waitKey(0);

            return 0;
        }

        cv::imshow("Bezier Curve", window);
        key = cv::waitKey(20);
    }

    return 0;
}
