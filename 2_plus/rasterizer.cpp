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

//MSAA
static int insideTriangle(int x, int y, array<Vector3f,3> v)
{   
    // TODO : Implement this function to check if the point (x, y) is inside the triangle represented by _v[0], _v[1], _v[2]
    int num=0;
    for (float n = 0; n < 4; ++n) {
        int z[3]={0,0,0};
        Vector3f point;
        if (n==0) point<<x+0.33,y+0.33,0.f;
        else if (n==1) point<<x+0.33,y+0.67,0.f;
        else if (n==2) point<<x+0.67,y+0.33,0.f;
        else if (n==3) point<<x+0.67,y+0.67,0.f;
        for (int i = 0; i < 3; ++i) {
            Vector3f edge;
            edge=v[(i+1)%3]-v[i];
            Vector3f line;
            line=v[i]-point;
            Vector3f normal;
            normal=edge.cross(line);
            if(normal.z()>0)
                z[i]=1;
            else
                z[i]=-1;
        }
        if (z[0]==z[1]&&z[0]==z[2])
            num++;
    }
    return num;
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
    }
}

//Screen space rasterization
//MSAA
void rst::rasterizer::rasterize_triangle(const Triangle& t) {
    auto v = t.toVector4();

    int right,left,top,bottom;
    left=min(min(v[0].x(),v[1].x()),v[2].x());
    if(max(max(v[0].x(),v[1].x()),v[2].x())-(int)max(v[0].x(),v[1].x()),v[2].x()==0){right=max(max(v[0].x(),v[1].x()),v[2].x());}
    else{right=max(max(v[0].x(),v[1].x()),v[2].x())+1;}
    bottom=min(min(v[0].y(),v[1].y()),v[2].y());
    if(max(max(v[0].y(),v[1].y()),v[2].y())-(int)max(v[0].y(),v[1].y()),v[2].y()==0){top=max(max(v[0].y(),v[1].y()),v[2].y());}
    else{top=max(max(v[0].y(),v[1].y()),v[2].y())+1;}

    float buffer[right-left][top-bottom];
    for (int i = 0; i < right-left; ++i) {
        for (int j = 0; j < top-bottom; ++j) {
            buffer[i][j]=0x3f3f3f3f;
        }
    }

    std::array<Eigen::Vector3f, 3> res;
    std::transform(std::begin(v), std::end(v), res.begin(), [](auto& vec) { return Eigen::Vector3f(vec.x(), vec.y(),0.f); });

    for (int x = left; x < right; ++x) {
        for (int y = bottom; y < top; ++y) {
            int num=insideTriangle(x,y,res);
            if(num!=0) {
                auto [alpha, beta, gamma] = computeBarycentric2D(x, y, t.v);
                float w_reciprocal = 1.0/(alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
                float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
                z_interpolated *= w_reciprocal;
                if (z_interpolated<buffer[x-left][y-bottom]){
                    buffer[x-left][y-bottom]=z_interpolated;

                    Vector3f point;
                    point<<x,y,1;
                    if (num==4) set_pixel(point,t.getColor(1));
                    else if (num==3) set_pixel(point,t.getColor(0.875));
                    else if (num==2) set_pixel(point,t.getColor(0.5));
                    else if (num==1) set_pixel(point,t.getColor(0.125));
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