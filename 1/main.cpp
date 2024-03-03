#include "Triangle.hpp"
#include "rasterizer.hpp"
#include <Eigen/Eigen>
#include <iostream>
#include <opencv2/opencv.hpp>

constexpr double MY_PI = 3.1415926535898;
Eigen::Matrix4f get_view_matrix(Eigen::Vector3f eye_pos)
{
    Eigen::Matrix4f view = Eigen::Matrix4f::Identity();

    Eigen::Matrix4f translate;
    translate <<
        1, 0, 0, -eye_pos[0],
        0, 1, 0, -eye_pos[1],
        0, 0, 1, -eye_pos[2],
        0, 0, 0,           1;

    view = translate * view;

    return view;
}

Eigen::Matrix4f get_model_matrix(float rotation_angle)
{
    Eigen::Matrix4f model = Eigen::Matrix4f::Identity();

    // TODO: Implement this function
    // Create the model matrix for rotating the triangle around the Z axis.
    // Then return it.

    Eigen::Matrix4f rotation;
    float sinAngle = sin(rotation_angle*MY_PI/180.0), cosAngle = cos(rotation_angle*MY_PI/180.0);
    rotation <<
        cosAngle, -sinAngle, 0, 0,
        sinAngle, cosAngle, 0, 0,
        0, 0, 1, 0,
        0, 0, 0, 1;

    model *= rotation;

    return model;
}

//������ݣ��õ��������ԭ��������ת�任����
Eigen::Matrix4f get_rotation(Vector3f axis,float angle)
{
    //��ʼ������
    Eigen::Matrix4f model = Eigen::Matrix4f::Identity();

    //ͨ���޵����˹��ʽ�ó�
    Eigen::Matrix3f temp = Eigen::Matrix3f::Identity();
    float ag = angle/180*MY_PI;
    Eigen::Matrix3f tr;
    Eigen::Matrix3f mul;
    mul <<
            0, -axis[2], axis[1],
            axis[2], 0, -axis[0],
            -axis[1], axis[0], 0;
    tr = cos(ag)*temp + (1-cos(ag))*axis*axis.adjoint() + mul*sin(ag);
    model << tr(0,0), tr(0,1), tr(0,2), 0,
            tr(1,0), tr(1,1), tr(1,2), 0,
            tr(2,0), tr(2,1), tr(2,2), 0,
            0, 0, 0, 1;

    return model;
}

Eigen::Matrix4f get_projection_matrix(float eye_fov, float aspect_ratio,
                                      float zNear, float zFar)
{
    // Students will implement this function

    Eigen::Matrix4f projection = Eigen::Matrix4f::Identity();

    // TODO: Implement this function
    // Create the projection matrix for the given parameters.
    // Then return it.

    Eigen::Matrix4f persp2ortho = Eigen::Matrix4f::Identity();
    persp2ortho << 
        -zNear, 0, 0, 0,
        0, -zNear, 0, 0,
        0, 0, -(zNear + zFar), -zNear * zFar,
        0, 0, 1, 0;

    double halfEyeRadian = eye_fov * MY_PI / 2 / 180.0;
    double top = zNear * tan(halfEyeRadian);
    double bottom = -top;
    double right = top * aspect_ratio;
    double left = -right;

    Eigen::Matrix4f orthoScale = Eigen::Matrix4f::Identity();
    orthoScale << 2 / (right - left), 0, 0, 0,
        0, 2 / (top - bottom), 0, 0,
        0, 0, 2 / (zNear - zFar), 0,
        0, 0, 0, 1;

    Eigen::Matrix4f orthoTrans = Eigen::Matrix4f::Identity();
    orthoTrans << 1, 0, 0, -(right + left) / 2,
        0, 1, 0, -(top + bottom) / 2,
        0, 0, 1, -(zNear + zFar) / 2,
        0, 0, 0, 1;

    Eigen::Matrix4f matrixOrtho = orthoScale * orthoTrans;

    projection = matrixOrtho * persp2ortho;

    return projection;
}

int main(int argc, const char** argv)
{
    float angle = 0;
    bool command_line = false;
    std::string filename = "output.png";

    if (argc >= 3) {
        command_line = true;
        angle = std::stof(argv[2]); // -r by default
        if (argc == 4) {
            filename = std::string(argv[3]);
        }
        else
            return 0;
    }

    rst::rasterizer r(700, 700);

    Eigen::Vector3f eye_pos = {0, 0, 5};

    std::vector<Eigen::Vector3f> pos{{2, 0, -2}, {0, 2, -2}, {-2, 0, -2}};

    std::vector<Eigen::Vector3i> ind{{0, 1, 2}};

    auto pos_id = r.load_positions(pos);
    auto ind_id = r.load_indices(ind);

    int key = 0;
    int frame_count = 0;

    if (command_line) {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        r.set_model(get_model_matrix(angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

        r.draw(pos_id, ind_id, rst::Primitive::Triangle);
        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);

        cv::imwrite(filename, image);

        return 0;
    }

    while (key != 27) {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        r.set_model(get_model_matrix(angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

        r.draw(pos_id, ind_id, rst::Primitive::Triangle);

        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);
        cv::imshow("image", image);
        key = cv::waitKey(10);

        std::cout << "frame count: " << frame_count++ << '\n';

        if (key == 'a') {
            angle += 1;
        }
        else if (key == 'd') {
            angle -= 1;
        }
    }

    return 0;
}
