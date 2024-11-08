#include "map_beautify.hpp"
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Constrained_Delaunay_triangulation_2.h>
#include <CGAL/Delaunay_mesh_size_criteria_2.h>
#include <CGAL/Polygon_2.h>
#include <vector>

typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
typedef CGAL::Triangulation_vertex_base_2<K> Vb;
typedef CGAL::Constrained_triangulation_face_base_2<K> Fb;
typedef CGAL::Triangulation_data_structure_2<Vb, Fb> TDS;
typedef CGAL::Exact_predicates_tag Itag;
typedef CGAL::Constrained_Delaunay_triangulation_2<K, TDS, Itag> CDT;
typedef CDT::Point Point;

using namespace std;

int main() {
	// Step 1: Load image and preprocess
	string file_name = "C:\\Users\\Belfast\\OneDrive\\Desktop\\工作\\类云鲸美化\\dataset\\20240925_143825_17272463058305426_observe_show_buffer.png";
	cv::Mat img = cv::imread(file_name, cv::IMREAD_GRAYSCALE);
	if (img.empty()) {
		std::cerr << "Failed to load image" << std::endl;
		return -1;
	}

	// Convert to binary image 转换
	cv::Mat binary;
	cv::threshold(img, binary, 200, 255, cv::THRESH_BINARY);

	// Find contours
	std::vector<std::vector<cv::Point>> contours;
	cv::findContours(binary, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

	keepLargestContour(contours);

	// Ensure there's only one contour (single polygon) 确保只有一个轮廓（单个多边形）
	if (contours.size() != 1) {
		std::cerr << "Expected a single polygon in the image." << std::endl;
		return -1;
	}

	// Convert OpenCV points to CGAL points
	std::vector<Point> cgal_points;
	for (const auto& pt : contours[0]) {
		cgal_points.emplace_back(pt.x, pt.y);
	}

	// Step 2: Create and insert the polygon into the Constrained Delaunay Triangulation (CDT)
	CDT cdt;
	for (size_t i = 0; i < cgal_points.size(); ++i) {
		cdt.insert_constraint(cgal_points[i], cgal_points[(i + 1) % cgal_points.size()]);
	}

	// Step 3: Retrieve the triangles from the CDT
	std::vector<std::vector<cv::Point>> triangles;
	for (auto fit = cdt.finite_faces_begin(); fit != cdt.finite_faces_end(); ++fit) {
		cv::Point v1(fit->vertex(0)->point().x(), fit->vertex(0)->point().y());
		cv::Point v2(fit->vertex(1)->point().x(), fit->vertex(1)->point().y());
		cv::Point v3(fit->vertex(2)->point().x(), fit->vertex(2)->point().y());
		triangles.push_back({ v1, v2, v3 });
	}

	// Step 4: Draw the triangles on a new image
	cv::Mat output_img = cv::Mat::zeros(img.size(), CV_8UC3);
	for (const auto& triangle : triangles) {
		cv::polylines(output_img, triangle, true, cv::Scalar(255, 255, 255), 1);
	}

	// Display and save the result
	cv::imshow("Triangulation", output_img);
	imwrite_mdy_private(output_img, "triangulated_polygon.png");
	cv::waitKey(0);
	cv::destroyAllWindows();

	return 0;
}
