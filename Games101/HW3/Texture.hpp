//
// Created by LEI XU on 4/27/19.
//

#ifndef RASTERIZER_TEXTURE_H
#define RASTERIZER_TEXTURE_H
#include "global.hpp"
#include <eigen3/Eigen/Eigen>
#include <opencv2/opencv.hpp>
using namespace Eigen;
class Texture{
private:
    cv::Mat image_data;
    using V3 = cv::Vec<int,3>;
    using V3f = cv::Vec<float, 3>;
public:
    cv::Vec3b Lerp(cv::Vec3b A, cv::Vec3b B,float w)
    {
        if (w < 0)
            w = -w;

        cv::Vec3b res;
        for(int i=0;i<3;++i)
        {
            res[i] = (1 - w) * A[i] + w * B[i];
        }

        return res;
    }
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
        auto color = image_data.at<cv::Vec3b>(v_img, u_img);
        return Eigen::Vector3f(color[0], color[1], color[2]);
    }

    Eigen::Vector3f SampleTex(Vector2f uv)
    {
        auto u_img = uv.x() * width;
        auto v_img = (1 - uv.y()) * height;
        auto color = image_data.at<cv::Vec3b>(v_img, u_img);
        return Eigen::Vector3f(color[0], color[1], color[2]);
    }

    void Clamp2f(float min,float max,Vector2f& val)
    {
        val[0] = Clamp(0, 1, val[0]);
        val[1] = Clamp(0, 1, val[1]);
    }
    Eigen::Vector3f getColorBilinear(float u, float v)
    {

        //auto u_img = u * width;
        //auto v_img = (1 - v) * height;
        //auto color = image_data.at<cv::Vec3b>(v_img, u_img);
        //return Eigen::Vector3f(color[0], color[1], color[2]);

        float ui = 1.0f / width;
        float vi = 1.0f / height;
        Vector2f leftTop(u,v);
        Clamp2f(0,1, leftTop);

        Vector2f rightTop(u + ui, v);
        Clamp2f(0, 1, rightTop);

        Vector2f rightBottom(u + ui, v-vi);
        Clamp2f(0, 1, rightBottom);

        Vector2f leftBottom(u - ui, v-vi);
        Clamp2f(0, 1, leftBottom);

        Eigen::Vector3f A, B, C, D;

        A = SampleTex(leftTop);
        B = SampleTex(rightTop);
        C = SampleTex(rightBottom);
        D = SampleTex(leftBottom);

        float w = rightTop.x() - leftTop.x();
        float h = rightTop.y() - rightBottom.y();
        Eigen::Vector3f rt= (u - leftTop.x())/w*B+ (rightTop.x() - u) / w  * A;
        Eigen::Vector3f rb = (u - leftTop.x()) / w *C + (rightTop.x() - u) / w  * D;

        Eigen::Vector3f r = (v - leftBottom.y()) / h  * rt + (leftTop.y()-v) / h  * rb;

        return r;
    }

};
#endif //RASTERIZER_TEXTURE_H
