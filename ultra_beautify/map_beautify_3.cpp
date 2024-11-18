#include "map_beautify.hpp"
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Constrained_Delaunay_triangulation_2.h>
#include <CGAL/Delaunay_mesh_size_criteria_2.h>
#include <CGAL/mark_domain_in_triangulation.h>
#include <CGAL/Polygon_2.h>
#include <vector>

#include <unordered_map>
#include <boost/property_map/property_map.hpp>

typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
typedef CGAL::Triangulation_vertex_base_2<K> Vb;
typedef CGAL::Constrained_triangulation_face_base_2<K> Fb;
typedef CGAL::Triangulation_data_structure_2<Vb, Fb> TDS;
typedef CGAL::Exact_predicates_tag Itag;
typedef CGAL::Constrained_Delaunay_triangulation_2<K, TDS, Itag> CDT;
typedef CDT::Point Point;
typedef CDT::Face_handle Face_handle;
typedef CGAL::Polygon_2<K> Polygon_2;

using namespace std;


int orthogonal_bounding_rectangle(vector<cv::Point>& contour)
{
    cv::Rect boundingBox = cv::boundingRect(contour);

    // 计算矩形的四个角点
    std::vector<cv::Point> rectangleContour;
    rectangleContour.push_back(boundingBox.tl()); // 左上角
    rectangleContour.push_back(cv::Point(boundingBox.x, boundingBox.y + boundingBox.height - 1)); // 左下角
    rectangleContour.push_back(cv::Point(boundingBox.x + boundingBox.width - 1, boundingBox.y + boundingBox.height - 1)); // 右下角
    rectangleContour.push_back(cv::Point(boundingBox.x + boundingBox.width - 1, boundingBox.y)); // 右上角

    contour = rectangleContour;

    return 0;
}

static int
erode_contour(vector<cv::Point>& contour, int h, int w)
{
    cv::Mat img = cv::Mat::zeros(h, w, CV_8UC1);
    cv::drawContours(img, vector<vector<cv::Point>>{contour}, -1, cv::Scalar(255), cv::FILLED);

    cv::Mat kernel3 = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
    cv::morphologyEx(img, img, cv::MORPH_ERODE, kernel3);

    vector<vector<cv::Point>> ec;
    cv::findContours(img, ec, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    keepLargestContour(ec);

    contour = ec[0];

    return 0;
}

static int
observe_contour(const vector<cv::Point>& contour, int h, int w)
{
    cv::Mat img(h, w, CV_8UC3, cv::Scalar(255, 255, 255));

    cv::drawContours(img, vector<vector<cv::Point>>{ contour }, -1, cv::Scalar(0, 255, 0), 1);
    for (const auto& p : contour)
    {
        img.at<cv::Vec3b>(p) = cv::Vec3b(0, 0, 255);
    }

    imwrite_mdy_private(img, "obs_contour");

    return 0;
}

int remove_point(cv::Mat& img)
{
    auto cache = img.clone();
    int h = img.rows;
    int w = img.cols;

    for (int i = 0; i < h; i++)
    {
        for (int j = 0; j < w; j++)
        {
            if (cache.at<uchar>(i, j) == 255)
            {
                int p[8];
                p[0] = cache.at<uchar>(i, j - 1);
                p[1] = cache.at<uchar>(i + 1, j - 1);
                p[2] = cache.at<uchar>(i + 1, j);
                p[3] = cache.at<uchar>(i + 1, j + 1);
                p[4] = cache.at<uchar>(i, j + 1);
                p[5] = cache.at<uchar>(i - 1, j + 1);
                p[6] = cache.at<uchar>(i - 1, j);
                p[7] = cache.at<uchar>(i - 1, j - 1);

                int num_4 = (p[0] + p[2] + p[4] + p[6]) / 255;
                int num_8 = (p[1] + p[3] + p[5] + p[7]) / 255;

                int score = num_4 * 2 + num_8;
                if (score < 4)
                {
                    img.at<uchar>(i, j) = 0;
                }

            }
        }
    }

    return 0;
}

static int
single_approx_cdt_and_dilate(vector<cv::Point>& contour, double epsilon, int h, int w)
{
    observe_contour(contour, h, w);

    cv::approxPolyDP(contour, contour, epsilon, true);
    observe_contour(contour, h, w);

    Polygon_2 cgal_contour;

    for (const auto& p : contour)
    {
        cgal_contour.push_back(Point(p.x, p.y));
    }

    CDT cdt;
    cdt.insert_constraint(cgal_contour.vertices_begin(), cgal_contour.vertices_end(), true);

    std::unordered_map<Face_handle, bool> in_domain_map;
    boost::associative_property_map<std::unordered_map<Face_handle, bool>>
        in_domain(in_domain_map);

    //Mark facets that are inside the domain bounded by the polygon
    CGAL::mark_domain_in_triangulation(cdt, in_domain);

    // Step 3: Retrieve the triangles from the CDT
    std::vector<std::vector<cv::Point>> triangles;
    for (auto fit = cdt.finite_faces_begin(); fit != cdt.finite_faces_end(); ++fit)
    {
        if (get(in_domain, fit))
        {
            cv::Point v1(fit->vertex(0)->point().x(), fit->vertex(0)->point().y());
            cv::Point v2(fit->vertex(1)->point().x(), fit->vertex(1)->point().y());
            cv::Point v3(fit->vertex(2)->point().x(), fit->vertex(2)->point().y());
            triangles.push_back({ v1, v2, v3 });
        }
    }

    cv::Mat final_img = cv::Mat::zeros(h, w, CV_8UC1);
    cv::Mat observe_triangulation = cv::Mat::zeros(h, w, CV_8UC1);
    for (const auto& triangle : triangles)
    {
        auto tri = triangle;

        cv::polylines(observe_triangulation, tri, true, 255, 1);

        orthogonal_bounding_rectangle(tri);
        cv::fillPoly(final_img, tri, cv::Scalar(255, 255, 255));
    }

    vector<vector<cv::Point>> final_contours;
    cv::findContours(final_img, final_contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    contour = final_contours[0];

    observe_contour(contour, h, w);
    imwrite_mdy_private(observe_triangulation, "observe_triangulation");

    return 0;
}

int main() {
    // Step 1: Load image and preprocess
    string file_name = "C:\\Users\\Belfast\\OneDrive\\Desktop\\工作\\类云鲸美化\\dataset\\20240925_143825_17272463058305426_observe_show_buffer.png";
    cv::Mat img = cv::imread(file_name, cv::IMREAD_GRAYSCALE);
    if (img.empty()) {
        std::cerr << "Failed to load image" << std::endl;
        return -1;
    }

    int h = img.rows;
    int w = img.cols;

    // Convert to binary image 转换
    cv::Mat binary;
    cv::threshold(img, binary, 200, 255, cv::THRESH_BINARY);

    cv::Mat kernel3 = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
    //cv::morphologyEx(binary, binary, cv::MORPH_ERODE, kernel3);

    // Find contours
    cv::Mat cache = binary.clone();
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(cache, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    cache.release();

    keepLargestContour(contours);

    // Ensure there's only one contour (single polygon) 确保只有一个轮廓（单个多边形）
    if (contours.size() != 1) {
        std::cerr << "Expected a single polygon in the image." << std::endl;
        return -1;
    }

    //observe_contour(contours[0], h, w);

    cv::Mat frame = cv::Mat::zeros(img.size(), CV_8UC1);
    cv::drawContours(frame, contours, -1, 255, -1);

    cv::Mat obs = cv::Mat::zeros(img.size(), CV_8UC1);
    cv::subtract(frame, binary, obs);
    std::vector<std::vector<cv::Point>> obs_contours;
    std::vector<std::vector<cv::Point>> obs_rect;
    cv::findContours(obs, obs_contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    obs.release();

    for (const auto& oc : obs_contours)
    {
        auto carea = cv::contourArea(oc);
        if (carea >= 20)
        {
            if (carea > 100)
            {
                auto rect_o = oc;
                single_approx_cdt_and_dilate(rect_o, 3, h, w);
                obs_rect.push_back(rect_o);
            }
            else
            {
                auto rect_o = oc;
                orthogonal_bounding_rectangle(rect_o);
                obs_rect.push_back(rect_o);
            }
        }
    }

    {
        frame.setTo(0);
        erode_contour(contours[0], h, w);
        single_approx_cdt_and_dilate(contours[0], 5.5, h, w);
        cv::drawContours(frame, contours, -1, 255, -1);
    }
    //观察最后的多边形
    cv::Mat observe_before = cv::Mat::zeros(img.size(), CV_8UC1);
    cv::Mat obs_rect_img = cv::Mat::zeros(img.size(), CV_8UC1);
    cv::drawContours(obs_rect_img, obs_rect, -1, 255, -1);
    cv::subtract(frame, obs_rect_img, observe_before);
    obs_rect_img.release();
    cv::imshow("observe_before", observe_before);
    imwrite_mdy_private(observe_before, "observe_before");
    cv::waitKey(1);


    // Convert OpenCV points to CGAL points
    /*std::vector<Point> cgal_points;
    for (const auto& pt : contours[0]) {
        cgal_points.emplace_back(pt.x, pt.y);
    }
    std::vector<Polygon_2> cgal_obs;
    for (const auto& ob : obs_rect)
    {
        Polygon_2 cgal_ob;
        for (const auto& p : ob)
        {
            cgal_ob.push_back(Point(p.x, p.y));
        }
        cgal_obs.push_back(cgal_ob);
    }*/
    vector<vector<cv::Point>> contours_before;
    vector<cv::Vec4i> hierarchy;
    cv::findContours(observe_before, contours_before, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
    observe_before.release();
    vector<Polygon_2> cgal_before;
    for (const auto& cb : contours_before)
    {
        Polygon_2 sp;
        for (const auto& p : cb)
        {
            sp.push_back(Point(p.x, p.y));
        }
        cgal_before.push_back(sp);
    }

    // Step 2: Create and insert the polygon into the Constrained Delaunay Triangulation (CDT)
    CDT cdt;
    /*for (size_t i = 0; i < cgal_points.size(); ++i) {
        cdt.insert_constraint(cgal_points[i], cgal_points[(i + 1) % cgal_points.size()]);
    }
    for (const auto& cgalo : cgal_obs)
    {
        cdt.insert_constraint(cgalo.vertices_begin(), cgalo.vertices_end(), true);
    }*/
    for (const auto& cgalb : cgal_before)
    {
        cdt.insert_constraint(cgalb.vertices_begin(), cgalb.vertices_end(), true);
    }


    std::unordered_map<Face_handle, bool> in_domain_map;
    boost::associative_property_map<std::unordered_map<Face_handle, bool>>
        in_domain(in_domain_map);

    //Mark facets that are inside the domain bounded by the polygon
    CGAL::mark_domain_in_triangulation(cdt, in_domain);

    // Step 3: Retrieve the triangles from the CDT
    std::vector<std::vector<cv::Point>> triangles;
    for (auto fit = cdt.finite_faces_begin(); fit != cdt.finite_faces_end(); ++fit)
    {
        if (get(in_domain, fit))
        {
            cv::Point v1(fit->vertex(0)->point().x(), fit->vertex(0)->point().y());
            cv::Point v2(fit->vertex(1)->point().x(), fit->vertex(1)->point().y());
            cv::Point v3(fit->vertex(2)->point().x(), fit->vertex(2)->point().y());
            triangles.push_back({ v1, v2, v3 });
        }
    }

    // Step 4: Draw the triangles on a new image
    cv::Mat output_img = cv::Mat::zeros(img.size(), CV_8UC1);
    cv::Mat final_img = cv::Mat::zeros(img.size(), CV_8UC1);
    for (const auto& triangle : triangles)
    {
        auto tri = triangle;

        cv::polylines(output_img, tri, true, 255, 1);

        orthogonal_bounding_rectangle(tri);
        cv::fillPoly(final_img, tri, cv::Scalar(255, 255, 255));
    }

    //remove_point(output_img);

    // Display and save the result
    cv::imshow("Triangulation", output_img);
    imwrite_mdy_private(output_img, "triangulated_polygon.png");
    imwrite_mdy_private(final_img, "final.png");
    cv::waitKey(1);
    cv::destroyAllWindows();



    return 0;
}



