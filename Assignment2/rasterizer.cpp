// clang-format off
//
// Created by goksu on 4/6/19.
//

#include <algorithm>
#include <vector>
#include "rasterizer.hpp"
#include <opencv2/opencv.hpp>
#include <math.h>


rst::pos_buf_id rst::rasterizer::load_positions(const std::vector<Eigen::Vector3f> &positions)
{
    auto id = get_next_id();
    pos_buf.emplace(id, positions);

    return {id};
}

rst::ind_buf_id rst::rasterizer::load_indices(const std::vector<Eigen::Vector3i> &indices)
{
    auto id = get_next_id();
    ind_buf.emplace(id, indices);

    return {id};
}

rst::col_buf_id rst::rasterizer::load_colors(const std::vector<Eigen::Vector3f> &cols)
{
    auto id = get_next_id();
    col_buf.emplace(id, cols);

    return {id};
}

auto to_vec4(const Eigen::Vector3f& v3, float w = 1.0f)
{
    return Vector4f(v3.x(), v3.y(), v3.z(), w);
}


static bool insideTriangle(float x, float y, const Vector3f* _v)
{
    // 测试点是否在三角形内。你可以修改此函数的定义，这意味着，你可以按照自己的方式更新返回类型或函数参数
    // 该点与三角形三个顶点连成向量分别叉积，若叉积的 z 坐标都相同则在三角形内
    // TODO : Implement this function to check if the point (x, y) is inside the triangle represented by _v[0], _v[1], _v[2]

    // 0: negative; 1: positive
    int flag = -1;

    for(int i = 0; i < 3; i++) {
        // 待判断点
        Eigen::Vector3f p0 = {x, y, 0};
        // 1th 顶点
        Eigen::Vector3f p1 = _v[i];
        // 2th 顶点
        Eigen::Vector3f p2 = _v[(i+1)%3];

        Eigen::Vector3f v1 = p0 - p1;
        Eigen::Vector3f v2 = p2 - p1;

        // CrossProduct 的 z 坐标
        float CrossProduct = v1.cross(v2).z();

        // 在三角形边上
        if(CrossProduct == 0) continue;

        // 当前叉积的符号是否与前一个相同，否则返回false：在三角形外
        int sign = CrossProduct < 0 ? 0 : 1;
        if(flag == -1) flag = sign;
        if(flag != sign) return false;
    }

    return true;
}

static std::tuple<float, float, float> computeBarycentric2D(float x, float y, const Vector3f* v)
{
    float c1 = (x*(v[1].y() - v[2].y()) + (v[2].x() - v[1].x())*y + v[1].x()*v[2].y() - v[2].x()*v[1].y()) / (v[0].x()*(v[1].y() - v[2].y()) + (v[2].x() - v[1].x())*v[0].y() + v[1].x()*v[2].y() - v[2].x()*v[1].y());
    float c2 = (x*(v[2].y() - v[0].y()) + (v[0].x() - v[2].x())*y + v[2].x()*v[0].y() - v[0].x()*v[2].y()) / (v[1].x()*(v[2].y() - v[0].y()) + (v[0].x() - v[2].x())*v[1].y() + v[2].x()*v[0].y() - v[0].x()*v[2].y());
    float c3 = (x*(v[0].y() - v[1].y()) + (v[1].x() - v[0].x())*y + v[0].x()*v[1].y() - v[1].x()*v[0].y()) / (v[2].x()*(v[0].y() - v[1].y()) + (v[1].x() - v[0].x())*v[2].y() + v[0].x()*v[1].y() - v[1].x()*v[0].y());
    return {c1,c2,c3};
}

void rst::rasterizer::draw(pos_buf_id pos_buffer, ind_buf_id ind_buffer, col_buf_id col_buffer, Primitive type)
{
    auto& buf = pos_buf[pos_buffer.pos_id];
    auto& ind = ind_buf[ind_buffer.ind_id];
    auto& col = col_buf[col_buffer.col_id];

    float f1 = (50 - 0.1) / 2.0;
    float f2 = (50 + 0.1) / 2.0;

    Eigen::Matrix4f mvp = projection * view * model;
    for (auto& i : ind)
    {
        Triangle t;
        Eigen::Vector4f v[] = {
                mvp * to_vec4(buf[i[0]], 1.0f),
                mvp * to_vec4(buf[i[1]], 1.0f),
                mvp * to_vec4(buf[i[2]], 1.0f)
        };
        //Homogeneous division
        for (auto& vec : v) {
            vec /= vec.w();
        }
        //Viewport transformation
        for (auto & vert : v)
        {
            vert.x() = 0.5*width*(vert.x()+1.0);
            vert.y() = 0.5*height*(vert.y()+1.0);
            vert.z() = vert.z() * f1 + f2;
        }

        for (int i = 0; i < 3; ++i)
        {
            t.setVertex(i, v[i].head<3>());
            t.setVertex(i, v[i].head<3>());
            t.setVertex(i, v[i].head<3>());
        }

        auto col_x = col[i[0]];
        auto col_y = col[i[1]];
        auto col_z = col[i[2]];

        t.setColor(0, col_x[0], col_x[1], col_x[2]);
        t.setColor(1, col_y[0], col_y[1], col_y[2]);
        t.setColor(2, col_z[0], col_z[1], col_z[2]);

        rasterize_triangle(t);

        // downsampling
        for(int y=0; y<height; y++){
            for(int x=0; x<width; x++){
                Eigen::Vector3f color = {0., 0., 0.};
                float infinity = std::numeric_limits<float>::infinity();
                for(float j=start_point; j<1.0; j+=pixel_size_sm){
                    for(float i=start_point; i<1.0; i+=pixel_size_sm){
                        int index = get_index_ssaa(x, y, i ,j);
                        color += frame_buf_ssaa[index];
                    }
                }
                Eigen::Vector3f p;
                p << x, y, 0;
                set_pixel(p, color / (ssaa_h * ssaa_w));
            }
        }
    }
}

//Screen space rasterization
void rst::rasterizer::rasterize_triangle(const Triangle& t)
{
    // 执行三角形栅格化算法
    // 该函数的内部工作流程如下:
    // 1. 创建三角形的 2 维 bounding box。
    // 2. 遍历此 bounding box 内的所有像素(使用其整数索引)。
    //    然后，使用像素中心的屏幕空间坐标来检查中心点是否在三角形内。
    // 3. 如果在内部，则将其位置处的插值深度值 (interpolated depth value)
    //    与深度缓冲区 (depth buffer) 中的相应值进行比较。
    // 4. 如果当前点更靠近相机，请设置像素颜色并更新深度缓冲区 (depth buffer)。

    auto v = t.toVector4();
    
    // TODO : Find out the bounding box of current triangle.
    // iterate through the pixel and find if the current pixel is inside the triangle

    // If so, use the following code to get the interpolated z value.
    //auto[alpha, beta, gamma] = computeBarycentric2D(x, y, t.v);
    //float w_reciprocal = 1.0/(alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
    //float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
    //z_interpolated *= w_reciprocal;

    // TODO : set the current pixel (use the set_pixel function) to the color of the triangle (use getColor function) if it should be painted.

    // Bounding-box
    float x_min = 0, x_max = width, y_min = 0, y_max = height;

    for(int i=0; i<3; i++){
        x_min = std::min(v[i].x(), x_min);
        x_max = std::max(v[i].x(), x_max);
        y_min = std::min(v[i].y(), y_min);
        y_max = std::max(v[i].y(), y_max);
    }
    /*
    // Z-buffer
    for(int y=y_min; y<y_max; y++) {
        for(int x=x_min; x<x_max; x++) {
            if(insideTriangle(x+0.5, y+0.5, t.v)){
                // 重心插值，代码由框架给出
                auto[alpha, beta, gamma] = computeBarycentric2D(x+0.5, y+0.5, t.v);
                float w_reciprocal = 1.0/(alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
                float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
                z_interpolated *= w_reciprocal;

                // 将坐标位置转化为depth_buffer数组下标
                int index = get_index(x, y);
                // Z-buffer 算法，设置像素值，更新深度值
                // 注意这里 z 越大离视点越远
                if(z_interpolated < depth_buf[index]){
                    Eigen::Vector3f point;
                    point << x, y, z_interpolated;
                    set_pixel(point, t.getColor());
                    depth_buf[index] = z_interpolated;
                }
            }
        }
    }
     */

    // SSAA
    // Z-buffer
    for(int y=y_min; y<y_max; y++) {
        for(int x=x_min; x<x_max; x++) {
            for(float j=start_point; j<1.0; j+=pixel_size_sm) {
                for (float i = start_point; i < 1.0; i += pixel_size_sm) {
                    if (insideTriangle(x + i, y + j, t.v)) {
                        // 重心插值，代码由框架给出
                        auto [alpha, beta, gamma] = computeBarycentric2D(x + i, y + j, t.v);
                        float w_reciprocal = 1.0 / (alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
                        float z_interpolated =
                                alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
                        z_interpolated *= w_reciprocal;

                        // 将坐标位置转化为depth_buffer数组下标
                        int index = get_index_ssaa(x, y, i, j);
                        // Z-buffer 算法，设置像素值，更新深度值
                        // 注意这里 z 越大离视点越远
                        if (z_interpolated < depth_buf_ssaa[index]) {
                            frame_buf_ssaa[index] = t.getColor();
                            depth_buf_ssaa[index] = z_interpolated;
                        }
                    }
                }
            }
        }
    }


}

void rst::rasterizer::set_model(const Eigen::Matrix4f& m)
{
    model = m;
}

void rst::rasterizer::set_view(const Eigen::Matrix4f& v)
{
    view = v;
}

void rst::rasterizer::set_projection(const Eigen::Matrix4f& p)
{
    projection = p;
}

void rst::rasterizer::clear(rst::Buffers buff)
{
    if ((buff & rst::Buffers::Color) == rst::Buffers::Color)
    {
        std::fill(frame_buf.begin(), frame_buf.end(), Eigen::Vector3f{0, 0, 0});
        std::fill(frame_buf_ssaa.begin(), frame_buf_ssaa.end(), Eigen::Vector3f{0, 0, 0});
    }
    if ((buff & rst::Buffers::Depth) == rst::Buffers::Depth)
    {
        std::fill(depth_buf.begin(), depth_buf.end(), std::numeric_limits<float>::infinity());
        std::fill(depth_buf_ssaa.begin(), depth_buf_ssaa.end(), std::numeric_limits<float>::infinity());
    }
}

rst::rasterizer::rasterizer(int w, int h) : width(w), height(h)
{
    frame_buf.resize(w * h);
    depth_buf.resize(w * h);
    frame_buf_ssaa.resize(w * h * ssaa_w * ssaa_h);
    depth_buf_ssaa.resize(w * h * ssaa_w * ssaa_h);
}

int rst::rasterizer::get_index(int x, int y)
{
    return (height-1-y)*width + x;
}

int rst::rasterizer::get_index_ssaa(int x, int y, float i, float j)
{
    int ssaa_height = height * ssaa_h;
    int ssaa_width = width * ssaa_w;

    i = int((i-start_point) / pixel_size_sm);
    j = int((j-start_point) / pixel_size_sm);

    return (ssaa_height-1-y*ssaa_h+j) * ssaa_width + x*ssaa_w + i;
}

void rst::rasterizer::set_pixel(const Eigen::Vector3f& point, const Eigen::Vector3f& color)
{
    //old index: auto ind = point.y() + point.x() * width;
    auto ind = (height-1-point.y())*width + point.x();
    frame_buf[ind] = color;

}

// clang-format on