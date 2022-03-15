/// \file  PolygonGenerator.cpp
/// \brief Generate convex hull from point cloud cluster of detected object
/// \author Hatem Darweesh
/// \date Nov 2, 2016

#include "op_ros_helpers/PolygonGenerator.h"
#include "op_planner/PlanningHelpers.h"
#include <float.h>

namespace PlannerHNS
{

PolygonGenerator::PolygonGenerator(int nQuarters)
{
	m_Quarters = CreateQuarterViews(nQuarters);
}

PolygonGenerator::~PolygonGenerator()
{
}

std::vector<GPSPoint> PolygonGenerator::EstimateClusterPolygon(const pcl::PointCloud<pcl::PointXYZ>& cluster, const GPSPoint& original_centroid, GPSPoint& new_centroid, const double& polygon_resolution)
{
	for(unsigned int i=0; i < m_Quarters.size(); i++)
			m_Quarters.at(i).ResetQuarterView();

	WayPoint p;
	for(unsigned int i=0; i< cluster.points.size(); i++)
	{
		p.pos.x = cluster.points.at(i).x;
		p.pos.y = cluster.points.at(i).y;
		p.pos.z = original_centroid.z;

		GPSPoint v(p.pos.x - original_centroid.x , p.pos.y - original_centroid.y, 0, 0);
		p.distanceCost = pointNorm(v);
		p.pos.a = UtilityHNS::UtilityH::FixNegativeAngle(atan2(v.y, v.x))*(180. / M_PI);

		for(unsigned int j = 0 ; j < m_Quarters.size(); j++)
		{
			if(m_Quarters.at(j).UpdateQuarterView(p))
				break;
		}
	}

	m_Polygon.clear();
	WayPoint wp;
	for(unsigned int j = 0 ; j < m_Quarters.size(); j++)
	{
		if(m_Quarters.at(j).GetMaxPoint(wp))
			m_Polygon.push_back(wp.pos);
	}

//	//Fix Resolution:
	bool bChange = true;
	while (bChange && m_Polygon.size()>1)
	{
		bChange = false;
		GPSPoint p1 =  m_Polygon.at(m_Polygon.size()-1);
		for(unsigned int i=0; i< m_Polygon.size(); i++)
		{
			GPSPoint p2 = m_Polygon.at(i);
			double d = hypot(p2.y- p1.y, p2.x - p1.x);
			if(d > polygon_resolution)
			{
				GPSPoint center_p = p1;
				center_p.x = (p2.x + p1.x)/2.0;
				center_p.y = (p2.y + p1.y)/2.0;
				m_Polygon.insert(m_Polygon.begin()+i, center_p);
				bChange = true;
				break;
			}

			p1 = p2;
		}
	}
	GPSPoint sum_p;
	for(unsigned int i = 0 ; i< m_Polygon.size(); i++)
	{
		sum_p.x += m_Polygon.at(i).x;
		sum_p.y += m_Polygon.at(i).y;
	}

	new_centroid = original_centroid;

	if(m_Polygon.size() > 0)
	{
		new_centroid.x = sum_p.x / (double)m_Polygon.size();
		new_centroid.y = sum_p.y / (double)m_Polygon.size();
	}

	return m_Polygon;

}

std::vector<QuarterView> PolygonGenerator::CreateQuarterViews(const int& nResolution)
{
	std::vector<QuarterView> quarters;
	if(nResolution <= 0)
		return quarters;

	double range = 360.0 / nResolution;
	double angle = 0;
	for(int i = 0; i < nResolution; i++)
	{
		QuarterView q(angle, angle+range, i);
		quarters.push_back(q);
		angle+=range;
	}

	return quarters;
}

ConvexHull::ConvexHull()
{

}

std::vector<GPSPoint> ConvexHull::EstimateClusterHull(const pcl::PointCloud<pcl::PointXYZ>& cluster, const GPSPoint& original_centroid, const double& polygon_resolution)
{
	m_ConvexHull.clear();
	m_PointsList.clear();

	if(cluster.points.size() < 3) return m_ConvexHull;

	m_StartPoint.pos.y = DBL_MAX;
	int min_index = -1;
	WayPoint p;
	for(auto& pcl_p: cluster.points)
	{
		p.pos.x = pcl_p.x;
		p.pos.y = pcl_p.y;
		p.pos.z = original_centroid.z;
		if(p.pos.y < m_StartPoint.pos.y)
		{
			m_StartPoint = p;
			min_index = m_PointsList.size();
		}
		m_PointsList.push_back(p);
	}


	m_ConvexHull.push_back(m_StartPoint.pos);
	m_PointsList.erase(m_PointsList.begin()+min_index);

	WayPoint prev_point = m_StartPoint;
	double prev_angle = -1;
	while(1)
	{
		double min_angle = DBL_MAX;
		double max_d = 0;
		WayPoint next_p;
		int next_p_i = -1;

		for(unsigned int i=0; i < m_PointsList.size(); i++)
		{
			GPSPoint v(m_PointsList.at(i).pos.x - prev_point.pos.x, m_PointsList.at(i).pos.y - prev_point.pos.y, 0, 0);
			double distanceCost = pointNorm(v);
			double a = UtilityHNS::UtilityH::FixNegativeAngle(atan2(v.y, v.x))*(180. / M_PI);

			int comp_a = UtilityHNS::UtilityH::CompareDouble(a, min_angle);
			if(comp_a <= 0 && a > prev_angle)
			{
				if(comp_a < 0 || distanceCost > max_d)
				{
					min_angle = a;
					max_d = distanceCost;
					next_p = m_PointsList.at(i);
					next_p_i = i;
				}
			}
		}

		if(((next_p.pos.x == m_StartPoint.pos.x) && (next_p.pos.y == m_StartPoint.pos.y)) || (next_p_i < 0))
		{
			break;
		}

		m_ConvexHull.push_back(next_p.pos);


		GPSPoint v(next_p.pos.x - prev_point.pos.x, next_p.pos.y - prev_point.pos.y, 0, 0);
		prev_angle = UtilityHNS::UtilityH::FixNegativeAngle(atan2(v.y, v.x))*(180. / M_PI);
		prev_point = next_p;
		m_PointsList.erase(m_PointsList.begin()+next_p_i);
	}

	FixResolution(polygon_resolution);

	return m_ConvexHull;
}

void ConvexHull::FixResolution(double polygon_resolution)
{
	//Fix Resolution:
	bool bChange = true;
	while (bChange && m_ConvexHull.size()>1)
	{
		bChange = false;
		GPSPoint p1 =  m_ConvexHull.at(m_ConvexHull.size()-1);
		for(unsigned int i=0; i< m_ConvexHull.size(); i++)
		{
			GPSPoint p2 = m_ConvexHull.at(i);
			double d = hypot(p2.y- p1.y, p2.x - p1.x);
			if(d > polygon_resolution)
			{
				GPSPoint center_p = p1;
				center_p.x = (p2.x + p1.x)/2.0;
				center_p.y = (p2.y + p1.y)/2.0;
				m_ConvexHull.insert(m_ConvexHull.begin()+i, center_p);
				bChange = true;
				break;
			}

			p1 = p2;
		}
	}
}

} /* namespace PlannerXNS */
