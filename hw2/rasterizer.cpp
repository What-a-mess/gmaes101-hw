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
    // TODO : Implement this function to check if the point (x, y) is inside the triangle represented by _v[0], _v[1], _v[2]
    bool res[3];
    Vector3f side;
    Vector3f p(x, y, 1.0f);
    Vector3f vecP;
    for (int i = 0; i < 3; i++) {
        side = _v[i] - _v[(i + 1) % 3];
        // std::cout << side;
        vecP = p - _v[i];
        res[i] = (vecP[0] * side[1] - vecP[1] * side[0]) < 0;
    }
    return (res[0] == res[1] && res[1] == res[2]);
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
            float temp = vec.w();
            vec /= vec.w();
            vec.w() = temp;
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
    }
}

//Screen space rasterization
void rst::rasterizer::rasterize_triangle(const Triangle& t) {
    auto v = t.toVector4();

    int pixScale = 1;
    float step = 1.0 / pixScale;
    
    // TODO : Find out the bounding box of current triangle.
    // iterate through the pixel and find if the current pixel is inside the triangle
    int min_x = INT_MAX, max_x = 0, min_y = INT_MAX, max_y = 0;
    for (int i = 0; i < 3; i++) {
        if (v[i].x() < min_x) {
            min_x = v[i][0];
        }
        if (v[i].x() > max_x) {
            max_x = v[i][0];
        }
        if (v[i].y() < min_y) {
            min_y = v[i][1];
        }
        if (v[i].y() > max_y) {
            max_y = v[i][1];
        }
    }

    Vector3f tri_v[3];
    tri_v[0] << v[0].x(), v[0].y(), 1;
    tri_v[1] << v[1].x(), v[1].y(), 1;
    tri_v[2] << v[2].x(), v[2].y(), 1;

    for (int y = min_y; y <= max_y; y++) {
        for (int x = min_x; x <= max_x; x++) {
            if (insideTriangle(x, y, tri_v)) {
                auto[alpha, beta, gamma] = computeBarycentric2D(x, y, t.v);
                float w_reciprocal = 1.0/(alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
                float z_interpolated = alpha * v[0].w() / v[0].w() + beta * v[1].w() / v[1].w() + gamma * v[2].w() / v[2].w();
                // if (abs(z_interpolated) - 1 > 1e-8) {
                //     std::cout << "error\n";
                // }
                z_interpolated *= w_reciprocal;

                int index = get_index(x, y);
                if (depth_buf[index] > z_interpolated) {
                    depth_buf[index] = z_interpolated;
                    set_pixel(Vector3f(x, y, 1), t.getColor());
                }
            }
        }
    }

    // float x_arr[4] = {0.25, 0.25, 0.75, 0.75};
    // float y_arr[4] = {0.25, 0.75, 0.25, 0.75};

    // for (int cur_y = min_y; cur_y <= max_y; cur_y++) {
    //     for (int cur_x = min_x; cur_x <= max_x; cur_x++) {

    //         Vector3f color(0, 0, 0);
    //         int index = get_index(cur_x, cur_y);

    //         for (int i = 0; i < 4; i++) {
    //             float x = cur_x + x_arr[i];
    //             float y = cur_y + y_arr[i];
    //             if (insideTriangle(x, y, tri_v)) {

    //                 auto[alpha, beta, gamma] = computeBarycentric2D(x, y, t.v);
    //                 float w_reciprocal = 1.0/(alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
    //                 float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
                    
    //                 z_interpolated *= w_reciprocal;
    //                 if (w_reciprocal - 1 > 1e-5) std::cout << w_reciprocal << " " << alpha + beta + gamma << '\n';
    //                 if (z_buffer[index][i] > z_interpolated) {
    //                     z_buffer[index][i] = z_interpolated;
    //                     color_buffer[index][i] = t.getColor();
    //                 }
    //             }
    //         }

    //         color = (color_buffer[index][0] + color_buffer[index][1] + color_buffer[index][2] + color_buffer[index][3]) / 4;

    //         set_pixel(Vector3f(cur_x, cur_y, 1), color);
            
    //     }
    // }    

    // If so, use the following code to get the interpolated z value.
    //auto[alpha, beta, gamma] = computeBarycentric2D(x, y, t.v);
    //float w_reciprocal = 1.0/(alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
    //float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
    //z_interpolated *= w_reciprocal;

    // TODO : set the current pixel (use the set_pixel function) to the color of the triangle (use getColor function) if it should be painted.
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
    }
    if ((buff & rst::Buffers::Depth) == rst::Buffers::Depth)
    {
        std::fill(depth_buf.begin(), depth_buf.end(), std::numeric_limits<float>::infinity());
    }
}

rst::rasterizer::rasterizer(int w, int h) : width(w), height(h)
{
    frame_buf.resize(w * h);
    depth_buf.resize(w * h);
    // 初始化自定义的z-buffer与color-buffer
    z_buffer = std::vector<std::vector<float>>(w * h, std::vector<float>(4, std::numeric_limits<float>::infinity()));
    color_buffer = std::vector<std::vector<Vector3f>>(width * height, std::vector<Vector3f>(4, Vector3f(0, 0, 0)));
}

int rst::rasterizer::get_index(int x, int y)
{
    return (height-1-y)*width + x;
}

void rst::rasterizer::set_pixel(const Eigen::Vector3f& point, const Eigen::Vector3f& color)
{
    //old index: auto ind = point.y() + point.x() * width;
    auto ind = (height-1-point.y())*width + point.x();
    frame_buf[ind] = color;

}

// clang-format on