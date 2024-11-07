#include "map_beautify.hpp"
#include <vector>


#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Constrained_Delaunay_triangulation_2.h>
#include <CGAL/Delaunay_mesher_2.h>
#include <CGAL/Delaunay_mesh_face_base_2.h>
#include <CGAL/Delaunay_mesh_size_criteria_2.h>

typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
typedef CGAL::Triangulation_vertex_base_2<K> Vb;
typedef CGAL::Delaunay_mesh_face_base_2<K> Fb;
typedef CGAL::Triangulation_data_structure_2<Vb, Fb> Tds;

typedef CGAL::Constrained_Delaunay_triangulation_2<K, Tds> CDT;
typedef CGAL::Delaunay_mesh_size_criteria_2<CDT> Criteria;
typedef CDT::Vertex_handle Vertex_handle;
typedef CDT::Point Point;

using namespace std;



//三角化
void Triangulate()
{
    //找到边界上所有的像素点
    vector<cv::Point> ROIBoundPointList;
    //CalBoundPoint(ROIBoundPointList);

    CDT cdt;
    vector<Vertex_handle> vertexList;
    cout << ROIBoundPointList.size() << endl;

    for (int i = 0; i < ROIBoundPointList.size(); i++)
    {
        vertexList.push_back(cdt.insert(Point(ROIBoundPointList[i].x, ROIBoundPointList[i].y)));
    }

    for (unsigned int i = 0; i < vertexList.size() - 1; i++)
    {
        cdt.insert_constraint(vertexList[i], vertexList[i + 1]);
    }
    cdt.insert_constraint(vertexList[vertexList.size() - 1], vertexList[0]);


    std::cout << "Number of vertices: " << cdt.number_of_vertices() << std::endl;
    std::cout << "Meshing the triangulation..." << std::endl;

    //CGAL::refine_Delaunay_mesh_2(cdt, Criteria());

    // 设置细化标准并使用新的 refine_Delaunay_mesh_2 函数
    Criteria criteria(0.125, 0.5);  // 设置细化标准的参数（例如最大边长和形状质量参数）
    CGAL::refine_Delaunay_mesh_2(cdt, CGAL::parameters::criteria(criteria));


    std::cout << "Number of vertices: " << cdt.number_of_vertices() << std::endl;


    /*CDT::Face_iterator fit;
    for (fit = cdt.faces_begin(); fit != cdt.faces_end(); ++fit)
    {
        QVector<QPointF> triPoint;
        triPoint.push_back(QPointF(fit->vertex(0)->point().x(), fit->vertex(0)->point().y()));
        triPoint.push_back(QPointF(fit->vertex(1)->point().x(), fit->vertex(1)->point().y()));
        triPoint.push_back(QPointF(fit->vertex(2)->point().x(), fit->vertex(2)->point().y()));
        QPolygonF tri(triPoint);
        triList.push_back(tri);
    }

    bTri = true;
    update();*/

    // 遍历并输出细化后的三角形
    for (CDT::Face_iterator fit = cdt.faces_begin(); fit != cdt.faces_end(); ++fit)
    {
        if (cdt.is_infinite(fit)) continue;  // 忽略无限面

        std::cout << "Triangle vertices:" << std::endl;
        for (int i = 0; i < 3; i++) {
            std::cout << "(" << fit->vertex(i)->point().x() << ", "
                << fit->vertex(i)->point().y() << ")" << std::endl;
        }
        std::cout << "----" << std::endl;
    }
}




