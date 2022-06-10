// clang-format off
//
// Created by goksu on 4/6/19.
//

#include <algorithm>
#include <vector>
#include "rasterizer.hpp"
#include <opencv2/opencv.hpp>
#include <math.h>


rst::pos_buf_id rst::rasterizer::load_positions(const std::vector<Eigen::Vector3f>& positions)
{
	auto id = get_next_id();
	pos_buf.emplace(id, positions);

	return { id };
}

rst::ind_buf_id rst::rasterizer::load_indices(const std::vector<Eigen::Vector3i>& indices)
{
	auto id = get_next_id();
	ind_buf.emplace(id, indices);

	return { id };
}

rst::col_buf_id rst::rasterizer::load_colors(const std::vector<Eigen::Vector3f>& cols)
{
	auto id = get_next_id();
	col_buf.emplace(id, cols);

	return { id };
}

auto to_vec4(const Eigen::Vector3f& v3, float w = 1.0f)
{
	return Vector4f(v3.x(), v3.y(), v3.z(), w);
}


static bool insideTriangle(int x, int y, const Vector3f* _v)
{
	// TODO : Implement this function to check if the point (x, y) is inside the triangle represented by _v[0], _v[1], _v[2]
}

static std::tuple<float, float, float> computeBarycentric2D(float x, float y, const Vector3f* v)
{
	float c1 = (x * (v[1].y() - v[2].y()) + (v[2].x() - v[1].x()) * y + v[1].x() * v[2].y() - v[2].x() * v[1].y()) / (v[0].x() * (v[1].y() - v[2].y()) + (v[2].x() - v[1].x()) * v[0].y() + v[1].x() * v[2].y() - v[2].x() * v[1].y());
	float c2 = (x * (v[2].y() - v[0].y()) + (v[0].x() - v[2].x()) * y + v[2].x() * v[0].y() - v[0].x() * v[2].y()) / (v[1].x() * (v[2].y() - v[0].y()) + (v[0].x() - v[2].x()) * v[1].y() + v[2].x() * v[0].y() - v[0].x() * v[2].y());
	float c3 = (x * (v[0].y() - v[1].y()) + (v[1].x() - v[0].x()) * y + v[0].x() * v[1].y() - v[1].x() * v[0].y()) / (v[2].x() * (v[0].y() - v[1].y()) + (v[1].x() - v[0].x()) * v[2].y() + v[0].x() * v[1].y() - v[1].x() * v[0].y());
	return { c1,c2,c3 };
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
		for (auto& vert : v)
		{
			vert.x() = 2*0.5 *width * (vert.x() + 1.0);
			vert.y() = 2*0.5 *height * (vert.y() + 1.0);
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

		//SuperSample
		rasterize_triangle(t);
	}

	downSample();
}

#define Min(x,y) x>y? y:x;
#define Max(x,y) x<y? y:x;
//Screen space rasterization
void rst::rasterizer::rasterize_triangle(const Triangle& t) {
	auto v = t.toVector4();

	// TODO : Find out the bounding box of current triangle.
	BMin = t.v[0];
	BMax = t.v[0];
	for (int i = 0; i < 3; ++i)
	{
		BMin[0] = Min(t.v[i].x(), BMin[0]);
		BMin[1] = Min(t.v[i].y(), BMin[1]);
		BMin[2] = Min(t.v[i].z(), BMin[2]);

		BMax[0] = Max(t.v[i].x(), BMax[0]);
		BMax[1] = Max(t.v[i].y(), BMax[1]);
		BMax[2] = Max(t.v[i].z(), BMax[2]);
	}

	// iterate through the pixel and find if the current pixel is inside the triangle

	for (int y = BMin[1]; y < BMax[1]; ++y)
	{
		for (int x = BMin[0]; x < BMax[0]; ++x)
		{
			Vector3f pos(x, y, -1.0);


			Vector3f v1 = pos-t.v[0];
			v1.z() = -1.0f;
			Vector3f v2 = pos - t.v[1];
			v2.z() = -1.0f;
			Vector3f v3 = pos - t.v[2];
			v3.z() = -1.0f;

			Vector3f a= v1.cross(v2);
			Vector3f b = v2.cross(v3);
			Vector3f c = v3.cross(v1);

			if (a.dot(b) > 0 && b.dot(c) > 0 && c.dot(a) > 0)
			{
				// If so, use the following code to get the interpolated z value.
				std::tuple<float, float, float> bres;
				bres = computeBarycentric2D(x, y, t.v);
				float w_reciprocal = 1.0 / (std::get<0>(bres) / v[0].w() + std::get<1>(bres) / v[1].w() + std::get<2>(bres) / v[2].w());
				float z_interpolated = std::get<0>(bres) * v[0].z() / v[0].w() + std::get<1>(bres) * v[1].z() / v[1].w() + std::get<2>(bres) * v[2].z() / v[2].w();
				z_interpolated *= w_reciprocal;
				auto ind = (2*height - 1 - pos.y()) * 2*width + pos.x(); //support supersample

				if(z_interpolated<depthS_buf[ind])//support supersample
				{
					depthS_buf[ind] = z_interpolated;//support supersample
					set_Spixel(pos, t.getColor());//support supersample
				}

			}
			    
		}
	}

	downSample();

	// TODO : set the current pixel (use the set_pixel function) to the color of the triangle (use getColor function) if it should be painted.
}

void rst::rasterizer::downSample()
{
	int sHeight = 2 * height;
	int sWidth = 2 * width;
	for(int y1=0;y1<height;++y1)
	{
		for(int x1=0;x1<width;++x1)
		{
			Vector3f a, b, c, d;
			int idx = y1 * height + x1;

			int sa = 2 * y1 * sHeight + 2 * x1;
			int sb = 2 * y1 * sHeight + (2 * x1+1);
			int sc = (2 * y1+1) * sHeight + (2 * x1 + 1);
			int sd = (2 * y1 + 1) * sHeight + 2 * x1;
			a = frameS_buf[sa];
			b = frameS_buf[sb];
			c= frameS_buf[sc];
			d= frameS_buf[sd];
			frame_buf[idx] = (a + b + c + d) / 4.0f;
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
		std::fill(frame_buf.begin(), frame_buf.end(), Eigen::Vector3f{ 0, 0, 0 });
		std::fill(frameS_buf.begin(), frameS_buf.end(), Eigen::Vector3f{ 0, 0, 0 });
	}
	if ((buff & rst::Buffers::Depth) == rst::Buffers::Depth)
	{
		std::fill(depth_buf.begin(), depth_buf.end(), std::numeric_limits<float>::infinity());
		std::fill(depthS_buf.begin(), depthS_buf.end(), std::numeric_limits<float>::infinity());
	}
}

rst::rasterizer::rasterizer(int w, int h) : width(w), height(h)
{
	frameS_buf.resize(4 * w * h);
	frame_buf.resize(w * h);
	depth_buf.resize(w * h);
	depthS_buf.resize(4 * w * h);
}

int rst::rasterizer::get_index(int x, int y)
{
	return (height - 1 - y) * width + x;
}

void rst::rasterizer::set_pixel(const Eigen::Vector3f& point, const Eigen::Vector3f& color)
{
	//old index: auto ind = point.y() + point.x() * width;
	auto ind = (height - 1 - point.y()) * width + point.x();
	frame_buf[ind] = color;

}

void rst::rasterizer::set_Spixel(const Eigen::Vector3f& point, const Eigen::Vector3f& color)
{
	//old index: auto ind = point.y() + point.x() * width;
	auto ind = (2*height - 1 - point.y()) * 2*width + point.x();
	frameS_buf[ind] = color;
}

// clang-format on