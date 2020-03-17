/*
 * VectorMapLoader.cpp
 *
 *  Created on: Mar 16, 2020
 *      Author: hatem
 */

#include <op_planner/VectorMapLoader.h>
#include <op_planner/PlanningHelpers.h>

namespace PlannerHNS {
#define RIGHT_INITIAL_TURNS_COST 0
#define LEFT_INITIAL_TURNS_COST 0
#define DEFAULT_REF_VELOCITY 60 //km/h

VectorMapLoader::VectorMapLoader(int map_version, bool enable_lane_change, bool load_curbs, bool load_lines, bool load_wayareas) :
		_map_version(map_version), _find_parallel_lanes(enable_lane_change), _load_curbs(load_curbs), _load_lines(load_lines), _load_wayareas(load_wayareas) {
}

VectorMapLoader::~VectorMapLoader() {
}

void VectorMapLoader::SetAdditionalOrigin(const PlannerHNS::WayPoint& origin)
{
	_origin = origin;
}

void VectorMapLoader::LoadFromFile(const std::string& vectorMapFolder, PlannerHNS::RoadNetwork& map)
{
	map.Clear();
	UtilityHNS::MapRaw map_raw;
	map_raw.LoadFromFolder(vectorMapFolder);
	LoadFromData(map_raw, map);
}

void VectorMapLoader::LoadFromData(UtilityHNS::MapRaw& mapRaw, PlannerHNS::RoadNetwork& map)
{
	map.Clear();
	if(_map_version == 0)
	{
		ConstructRoadNetworkFromROSMessageVer0(mapRaw, _origin, map);
	}
	else
	{
		ConstructRoadNetworkFromROSMessage(mapRaw, _origin, map);
	}

	WayPoint origin = MappingHelpers::GetFirstWaypoint(map);
	std::cout << origin.pos.ToString();
}

void VectorMapLoader::ConstructRoadNetworkFromROSMessage(UtilityHNS::MapRaw& mapRaw, const PlannerHNS::WayPoint& origin, PlannerHNS::RoadNetwork& map)
{
	std::vector<Lane> roadLanes;
	for(unsigned int i=0; i< mapRaw.pLanes->m_data_list.size(); i++)
	{
		if(mapRaw.pLanes->m_data_list.at(i).LnID > RoadNetwork::g_max_lane_id)
			RoadNetwork::g_max_lane_id = mapRaw.pLanes->m_data_list.at(i).LnID;
	}

	std::cout << std::endl << " >> Extracting Lanes ... " << std::endl;
	CreateLanes(mapRaw.pLanes, mapRaw.pPoints, mapRaw.pNodes , roadLanes);

	std::cout << " >> Fix Waypoints errors ... " << std::endl;
	FixTwoPointsLanes(roadLanes);
	FixRedundantPointsLanes(roadLanes);
	ConnectLanes(mapRaw.pLanes, roadLanes);

	std::cout << " >> Create Missing lane connections ... " << std::endl;
	FixUnconnectedLanes(roadLanes);
	////FixTwoPointsLanes(roadLanes);

	//map has one road segment
	map.roadSegments.push_back(RoadSegment());
	map.roadSegments.at(0).Lanes = roadLanes;

	//Fix angle for lanes
	for(unsigned int rs = 0; rs < map.roadSegments.size(); rs++)
	{
		for(unsigned int i =0; i < map.roadSegments.at(rs).Lanes.size(); i++)
		{
			Lane* pL = &map.roadSegments.at(rs).Lanes.at(i);
			PlannerHNS::PlanningHelpers::FixAngleOnly(pL->points);
		}
	}

	//Link Lanes and lane's waypoints by pointers
	std::cout << " >> Link lanes and waypoints with pointers ... " << std::endl;
	MappingHelpers::LinkLanesPointers(map);

	for(unsigned int rs = 0; rs < map.roadSegments.size(); rs++)
	{
		for(unsigned int i =0; i < map.roadSegments.at(rs).Lanes.size(); i++)
		{
			Lane* pL = &map.roadSegments.at(rs).Lanes.at(i);
			for(unsigned int j = 0 ; j < pL->points.size(); j++)
			{
			    if(pL->points.at(j).actionCost.size() > 0)
			  {
				  if(pL->points.at(j).actionCost.at(0).first == LEFT_TURN_ACTION)
					{
					  AssignActionCostToLane(pL, LEFT_TURN_ACTION, LEFT_INITIAL_TURNS_COST);
					  break;
					}
				  else if(pL->points.at(j).actionCost.at(0).first == RIGHT_TURN_ACTION)
					{
					  AssignActionCostToLane(pL, RIGHT_TURN_ACTION, RIGHT_INITIAL_TURNS_COST);
					break;

					}
				}
			}
		}
	}

	if(_find_parallel_lanes)
	{
		std::cout << " >> Extract Lane Change Information... " << std::endl;
		MappingHelpers::FindAdjacentLanesV2(map);
	}

//	//Extract Signals and StopLines
//	std::cout << " >> Extract Signal data ... " << std::endl;
//	ExtractSignalDataV2(signal_data, vector_data, pPointsData, origin, map);
//
//	//Stop Lines
//	std::cout << " >> Extract Stop lines data ... " << std::endl;
//	ExtractStopLinesDataV2(stop_line_data, pLinedata, pPointsData, origin, map);
//
//	if(_load_lines)
//	{
//		//Lines
//		std::cout << " >> Extract lines data ... " << std::endl;
//		ExtractLines(pLinedata, pWhitelinesData, pPointsData, origin, map);
//	}
//
//	if(_load_curbs)
//	{
//		//Curbs
//		std::cout << " >> Extract curbs data ... " << std::endl;
//		ExtractCurbDataV2(curb_data, pLinedata, pPointsData, origin, map);
//	}
//
//	if(_load_wayareas)
//	{
//		//Wayarea
//		std::cout << " >> Extract wayarea data ... " << std::endl;
//		ExtractWayArea(area_data, wayarea_data, line_data, points_data, origin, map);
//	}
//
//	std::cout << " >> Connect Wayarea (boundaries) to waypoints ... " << std::endl;
//	ConnectBoundariesToWayPoints(map);
//	LinkBoundariesToWayPoints(map);
//
//	//Link waypoints
//	std::cout << " >> Link missing branches and waypoints... " << std::endl;
//	LinkMissingBranchingWayPointsV2(map);
//
//	//Link StopLines and Traffic Lights
//	std::cout << " >> Link StopLines and Traffic Lights ... " << std::endl;
//	LinkTrafficLightsAndStopLinesV2(map);
//	LinkTrafficLightsIntoGroups(map);
//	ConnectTrafficLightsAndStopLines(map);
//	ConnectTrafficSignsAndStopLines(map);

	std::cout << " >> Map loaded from data with " << roadLanes.size()  << " lanes, and " << map.lines.size() << " lines." << std::endl;
}

void VectorMapLoader::CreateLanes(UtilityHNS::AisanLanesFileReader* pLaneData, UtilityHNS::AisanPointsFileReader* pPointsData,
				UtilityHNS::AisanNodesFileReader* pNodesData, std::vector<PlannerHNS::Lane>& out_lanes)
{

	out_lanes.clear();
	std::vector<int> start_lines;
	GetLanesStartPoints(pLaneData, start_lines);
	for(unsigned int l =0; l < start_lines.size(); l++)
	{
		Lane _lane;
		GetLanePoints(pLaneData, pPointsData, pNodesData, start_lines.at(l), _lane);
		out_lanes.push_back(_lane);
	}
}

void VectorMapLoader::GetLanesStartPoints(UtilityHNS::AisanLanesFileReader* pLaneData,
				std::vector<int>& m_LanesStartIds)
{
	m_LanesStartIds.clear();
	UtilityHNS::AisanLanesFileReader::AisanLane* pL = nullptr;
	for(unsigned int il=0; il < pLaneData->m_data_list.size(); il++)
	{
		pL = &pLaneData->m_data_list.at(il);

		if(IsStartLanePoint(pLaneData, pL) == true)
		{
			m_LanesStartIds.push_back(pL->LnID);
		}
	}
}

bool VectorMapLoader::IsStartLanePoint(UtilityHNS::AisanLanesFileReader* pLaneData, UtilityHNS::AisanLanesFileReader::AisanLane* pL)
{
	//if(pL->JCT > 0 || pL->BLID == 0)
	if(pL->BLID == 0)
		return true;

	if(pL->BLID2 != 0 || pL->BLID3 != 0 || pL->BLID4 != 0)
		return true;

	UtilityHNS::AisanLanesFileReader::AisanLane* pPrevL = pLaneData->GetDataRowById(pL->BLID);
	if(pPrevL == nullptr || pPrevL->FLID2 > 0 || pPrevL->FLID3 > 0 || pPrevL->FLID4 > 0 || pPrevL->JCT > 0)
		return true;

	return false;
}

bool VectorMapLoader::IsEndLanePoint(UtilityHNS::AisanLanesFileReader* pLaneData, UtilityHNS::AisanLanesFileReader::AisanLane* pL)
{
	if(pL->FLID2 > 0 || pL->FLID3 > 0 || pL->FLID4 > 0)
		return true;

	UtilityHNS::AisanLanesFileReader::AisanLane* pNextL = pLaneData->GetDataRowById(pL->FLID);

	return IsStartLanePoint(pLaneData, pNextL);
}

void VectorMapLoader::GetLanePoints(UtilityHNS::AisanLanesFileReader* pLaneData,
			UtilityHNS::AisanPointsFileReader* pPointsData,
			UtilityHNS::AisanNodesFileReader* pNodesData, int lnID,
			PlannerHNS::Lane& out_lane)
{
	int next_lnid = lnID;
	bool bStart = false;
	bool bLast = false;
	int _rID = 0;
	out_lane.points.clear();
	UtilityHNS::AisanLanesFileReader::AisanLane* pL = nullptr;
	out_lane.id = lnID;
	out_lane.speed = 0;

	while(!bStart)
	{
		pL = pLaneData->GetDataRowById(next_lnid);
		if(pL == nullptr)
		{
			std::cout << "## Error, Can't find lane from ID: " << next_lnid << std::endl;
			break;
		}

		next_lnid = pL->FLID;
		if(next_lnid == 0)
		{
			bStart = true;
		}
		else
		{
			UtilityHNS::AisanLanesFileReader::AisanLane* pTempLanePointer = pLaneData->GetDataRowById(next_lnid);
			if(pTempLanePointer == nullptr)
			{
				std::cout << "Can't Find Lane:" << next_lnid << ", from Previous Lane = " << pL->LnID;
				return;
			}
			bStart = IsStartLanePoint(pLaneData, pTempLanePointer);
		}

//		if(_lnid == 1267 ) //|| _lnid == 1268 || _lnid == 1269 || _lnid == 958)
//			out_lane.id = lnID;

		if(out_lane.points.size() == 0)
		{
			WayPoint wp1;
			GetPointFromDataList(pPointsData, pNodesData->GetDataRowById(pL->BNID)->PID, wp1);
			wp1.v = pL->RefVel == 0 ? DEFAULT_REF_VELOCITY : pL->RefVel;
			wp1.id = pL->BNID;
			wp1.originalMapID = pL->LnID;
			wp1.laneId = lnID;
			out_lane.speed = pL->RefVel == 0 ? DEFAULT_REF_VELOCITY : pL->RefVel;

			if(pL->BLID != 0)
			{
				_rID = GetBeginPointIdFromLaneNo(pLaneData, pPointsData, pNodesData, pL->BLID);
				if(_rID > 0)
					wp1.fromIds.push_back(_rID);
			}
			if(pL->BLID2 != 0)
			{
				_rID = GetBeginPointIdFromLaneNo(pLaneData, pPointsData, pNodesData, pL->BLID2);
				if(_rID > 0)
					wp1.fromIds.push_back(_rID);
			}
			if(pL->BLID3 != 0)
			{
				_rID = GetBeginPointIdFromLaneNo(pLaneData, pPointsData, pNodesData, pL->BLID3);
				if(_rID > 0)
					wp1.fromIds.push_back(_rID);
			}
			if(pL->BLID4 != 0)
			{
				_rID = GetBeginPointIdFromLaneNo(pLaneData, pPointsData, pNodesData, pL->BLID4);
				if(_rID > 0)
					wp1.fromIds.push_back(_rID);
			}

			if(pL->LaneDir == 'L')
			{
				wp1.actionCost.push_back(std::make_pair(LEFT_TURN_ACTION, LEFT_INITIAL_TURNS_COST));
			}
			else  if(pL->LaneDir == 'R')
			{
				wp1.actionCost.push_back(std::make_pair(RIGHT_TURN_ACTION, RIGHT_INITIAL_TURNS_COST));
			}
			else
			{
				wp1.actionCost.push_back(std::make_pair(FORWARD_ACTION, 0));
			}

			WayPoint wp2;
			GetPointFromDataList(pPointsData, pNodesData->GetDataRowById(pL->FNID)->PID, wp2);
			wp2.v = pL->RefVel == 0 ? DEFAULT_REF_VELOCITY : pL->RefVel;
			wp2.id = pL->FNID;
			wp2.originalMapID = pL->LnID;
			wp2.laneId = lnID;
			wp2.fromIds.push_back(wp1.id);

			if(bStart && pL->FLID != 0)
			{
				_rID = GetEndPointIdFromLaneNo(pLaneData, pPointsData, pNodesData, pL->FLID);
				if(_rID > 0)
					wp2.toIds.push_back(_rID);
			}
			if(pL->FLID2 != 0)
			{
				_rID = GetEndPointIdFromLaneNo(pLaneData, pPointsData, pNodesData, pL->FLID2);
				if(_rID > 0)
					wp2.toIds.push_back(_rID);
			}
			if(pL->FLID3 != 0)
			{
				_rID = GetEndPointIdFromLaneNo(pLaneData, pPointsData, pNodesData, pL->FLID3);
				if(_rID > 0)
					wp2.toIds.push_back(_rID);
			}
			if(pL->FLID4 != 0)
			{
				_rID = GetEndPointIdFromLaneNo(pLaneData, pPointsData, pNodesData, pL->FLID4);
				if(_rID > 0)
					wp2.toIds.push_back(_rID);
			}

			if(pL->LaneDir == 'L')
			{
				wp2.actionCost.push_back(std::make_pair(LEFT_TURN_ACTION, LEFT_INITIAL_TURNS_COST));
			}
			else  if(pL->LaneDir == 'R')
			{
				wp2.actionCost.push_back(std::make_pair(RIGHT_TURN_ACTION, RIGHT_INITIAL_TURNS_COST));
			}
			else
			{
				wp2.actionCost.push_back(std::make_pair(FORWARD_ACTION, 0));
			}

			wp1.toIds.push_back(wp2.id);
			out_lane.points.push_back(wp1);
			out_lane.points.push_back(wp2);

		}
		else
		{
			WayPoint wp;
			UtilityHNS::AisanNodesFileReader::AisanNode* pTempNode = pNodesData->GetDataRowById(pL->FNID);
			if(pTempNode == nullptr)
			{
				std::cout << "Can't Find Node from Lane = " << pL->LnID << ", with FNID = " << pL->FNID;
				return;
			}

			GetPointFromDataList(pPointsData, pTempNode->PID, wp);
			wp.v = pL->RefVel == 0 ? DEFAULT_REF_VELOCITY : pL->RefVel;
			wp.id = pL->FNID;

			out_lane.points.at(out_lane.points.size()-1).toIds.push_back(wp.id);
			wp.fromIds.push_back(out_lane.points.at(out_lane.points.size()-1).id);

			if(bStart && pL->FLID != 0)
			{
				_rID = GetEndPointIdFromLaneNo(pLaneData, pPointsData, pNodesData, pL->FLID);
				if(_rID > 0)
					wp.toIds.push_back(_rID);
			}
			if(pL->FLID2 != 0)
			{
				_rID = GetEndPointIdFromLaneNo(pLaneData, pPointsData, pNodesData, pL->FLID2);
				if(_rID > 0)
					wp.toIds.push_back(_rID);
			}
			if(pL->FLID3 != 0)
			{
				_rID = GetEndPointIdFromLaneNo(pLaneData, pPointsData, pNodesData, pL->FLID3);
				if(_rID > 0)
					wp.toIds.push_back(_rID);
			}
			if(pL->FLID4 != 0)
			{
				_rID = GetEndPointIdFromLaneNo(pLaneData, pPointsData, pNodesData, pL->FLID4);
				if(_rID > 0)
					wp.toIds.push_back(_rID);
			}

			if(pL->LaneDir == 'L')
			{
				wp.actionCost.push_back(std::make_pair(LEFT_TURN_ACTION, LEFT_INITIAL_TURNS_COST));
			}
			else  if(pL->LaneDir == 'R')
			{
				wp.actionCost.push_back(std::make_pair(RIGHT_TURN_ACTION, RIGHT_INITIAL_TURNS_COST));
			}
			else
			{
				wp.actionCost.push_back(std::make_pair(FORWARD_ACTION, 0));
			}

			wp.originalMapID = pL->LnID;
			wp.laneId = lnID;

			if(IsPointExist(wp, out_lane.points))
				bStart = true;
			else
				out_lane.points.push_back(wp);
		}

//		if(next_lnid == 0)
//			break;
	}
}

bool VectorMapLoader::GetPointFromDataList(UtilityHNS::AisanPointsFileReader* pPointsData,const int& pid, WayPoint& out_wp)
{
	if(pPointsData == nullptr) return false;

	UtilityHNS::AisanPointsFileReader::AisanPoints* pP =  pPointsData->GetDataRowById(pid);

	if(pP!=nullptr)
	{
		out_wp.id = pP->PID;
		out_wp.pos.x = pP->Ly;
		out_wp.pos.y = pP->Bx;
		out_wp.pos.z = pP->H;
		out_wp.pos.lat = pP->L;
		out_wp.pos.lon = pP->B;
		out_wp.pos.alt = pP->H;
		MappingHelpers::correct_gps_coor(out_wp.pos.lat, out_wp.pos.lon);
		return true;
	}

	return false;
}

int VectorMapLoader::GetBeginPointIdFromLaneNo(UtilityHNS::AisanLanesFileReader* pLaneData, UtilityHNS::AisanPointsFileReader* pPointsData,
		UtilityHNS::AisanNodesFileReader* pNodesData,const int& LnID)
{
	if(pLaneData == nullptr) return false;
	UtilityHNS::AisanLanesFileReader::AisanLane* pL = nullptr;
	UtilityHNS::AisanPointsFileReader::AisanPoints* pP = nullptr;
	UtilityHNS::AisanNodesFileReader::AisanNode* pN = nullptr;

	pL = pLaneData->GetDataRowById(LnID);
	if(pL!=nullptr)
	{
		return pL->FNID;
	}

	return 0;
}

int VectorMapLoader::GetEndPointIdFromLaneNo(UtilityHNS::AisanLanesFileReader* pLaneData, UtilityHNS::AisanPointsFileReader* pPointsData,
		UtilityHNS::AisanNodesFileReader* pNodesData,const int& LnID)
{
	UtilityHNS::AisanLanesFileReader::AisanLane* pL = nullptr;
	UtilityHNS::AisanPointsFileReader::AisanPoints* pP = nullptr;
	UtilityHNS::AisanNodesFileReader::AisanNode* pN = nullptr;

	pL = pLaneData->GetDataRowById(LnID);
	if(pL!=nullptr)
	{
		return pL->BNID;
	}

	return 0;
}

bool VectorMapLoader::IsPointExist(const WayPoint& p, const std::vector<PlannerHNS::WayPoint>& points)
{
	for(unsigned int ip = 0; ip < points.size(); ip++)
	{
		if(points.at(ip).id == p.id)
		{
			return true;
		}
	}
	return false;
}

void VectorMapLoader::ConnectLanes(UtilityHNS::AisanLanesFileReader* pLaneData, std::vector<PlannerHNS::Lane>& lanes)
{
	for(unsigned int il = 0; il < lanes.size(); il++)
	{
		WayPoint fp = lanes.at(il).points.at(0);
		UtilityHNS::AisanLanesFileReader::AisanLane* pFirstL = pLaneData->GetDataRowById(fp.originalMapID);
		if(pFirstL!=nullptr)
		{
			if(pFirstL->BLID > 0)
				lanes.at(il).fromIds.push_back(pFirstL->BLID);
			if(pFirstL->BLID2 > 0)
				lanes.at(il).fromIds.push_back(pFirstL->BLID2);
			if(pFirstL->BLID3 > 0)
				lanes.at(il).fromIds.push_back(pFirstL->BLID3);
			if(pFirstL->BLID4 > 0)
				lanes.at(il).fromIds.push_back(pFirstL->BLID4);
		}

		WayPoint ep = lanes.at(il).points.at(lanes.at(il).points.size()-1);
		UtilityHNS::AisanLanesFileReader::AisanLane* pEndL = pLaneData->GetDataRowById(ep.originalMapID);
		if(pEndL!=nullptr)
		{
			if(pEndL->FLID > 0)
				lanes.at(il).toIds.push_back(pEndL->FLID);
			if(pEndL->FLID2 > 0)
				lanes.at(il).toIds.push_back(pEndL->FLID2);
			if(pEndL->FLID3 > 0)
				lanes.at(il).toIds.push_back(pEndL->FLID3);
			if(pEndL->FLID4 > 0)
				lanes.at(il).toIds.push_back(pEndL->FLID4);
		}
	}
}

void VectorMapLoader::FixRedundantPointsLanes(std::vector<Lane>& lanes)
{
	//Fix Redundant point for two points in a row
	for(unsigned int il=0; il < lanes.size(); il ++)
	{
		for(int ip = 1; ip < lanes.at(il).points.size(); ip++)
		{
			WayPoint* p1 = &lanes.at(il).points.at(ip-1);
			WayPoint* p2 = &lanes.at(il).points.at(ip);
			WayPoint* p3 = nullptr;
			if(ip+1 < lanes.at(il).points.size())
				p3 = &lanes.at(il).points.at(ip+1);

			double d = hypot(p2->pos.y-p1->pos.y, p2->pos.x-p1->pos.x);
			if(d == 0)
			{
				p1->toIds = p2->toIds;
				p1->originalMapID = p2->originalMapID;
				if(p3 != nullptr)
					p3->fromIds = p2->fromIds;

				lanes.at(il).points.erase(lanes.at(il).points.begin()+ip);
				ip--;
			}
		}
	}
}

void VectorMapLoader::FixTwoPointsLane(Lane& l)
{
	if(l.points.size() == 2)
	{
		RoadNetwork::g_max_point_id++;
		WayPoint wp = l.points.at(0);
		wp.id = RoadNetwork::g_max_point_id;
		wp.fromIds.clear();
		wp.fromIds.push_back(l.points.at(0).id);
		wp.toIds.clear();
		wp.toIds.push_back(l.points.at(1).id);

		l.points.at(0).toIds.clear();
		l.points.at(0).toIds.push_back(wp.id);

		l.points.at(1).fromIds.clear();
		l.points.at(1).fromIds.push_back(wp.id);

		wp.pos.x = (l.points.at(0).pos.x + l.points.at(1).pos.x)/2.0;
		wp.pos.y = (l.points.at(0).pos.y + l.points.at(1).pos.y)/2.0;
		wp.pos.z = (l.points.at(0).pos.z + l.points.at(1).pos.z)/2.0;

		l.points.insert(l.points.begin()+1, wp);
	}
	else if(l.points.size() < 2)
	{
		std::cout << "## WOW Lane " <<  l.id << " With Size (" << l.points.size() << ") " << std::endl;
	}
}

void VectorMapLoader::FixTwoPointsLanes(std::vector<Lane>& lanes)
{
	for(unsigned int il=0; il < lanes.size(); il ++)
	{
		for(unsigned int ip = 0; ip < lanes.at(il).points.size(); ip++)
		{
			if(lanes.at(il).points.at(ip).id > RoadNetwork::g_max_point_id)
			{
				RoadNetwork::g_max_point_id = lanes.at(il).points.at(ip).id;
			}
		}
	}

	for(unsigned int il=0; il < lanes.size(); il ++)
	{
		FixTwoPointsLane(lanes.at(il));
		PlannerHNS::PlanningHelpers::CalcAngleAndCost(lanes.at(il).points);
	}
}

void VectorMapLoader::FixUnconnectedLanes(std::vector<Lane>& lanes, const int& max_angle_diff)
{
	for(unsigned int il=0; il < lanes.size(); il ++)
	{
		PlanningHelpers::CalcAngleAndCost(lanes.at(il).points);
	}

	std::vector<Lane> sp_lanes = lanes;
	bool bAtleastOneChange = false;
	//Find before lanes
	for(unsigned int il=0; il < lanes.size(); il ++)
	{
		if(lanes.at(il).fromIds.size() == 0)
		{
			double closest_d = DBL_MAX;
			Lane* pL = nullptr;
			Lane* pFL = nullptr;
			for(int l=0; l < sp_lanes.size(); l ++)
			{
				if(lanes.at(il).id == sp_lanes.at(l).id)
				{
					pFL = &sp_lanes.at(l);
					break;
				}
			}

			PlannerHNS::RelativeInfo closest_info;
			int closest_index = -1;
			for(int l=0; l < sp_lanes.size(); l ++)
			{
				if(pFL->id != sp_lanes.at(l).id)
				{
					PlannerHNS::RelativeInfo info;
					WayPoint lastofother = sp_lanes.at(l).points.at(sp_lanes.at(l).points.size()-1);
					PlanningHelpers::GetRelativeInfoLimited(sp_lanes.at(l).points, pFL->points.at(0), info, 0);
					double back_distance = hypot(lastofother.pos.y - pFL->points.at(0).pos.y, lastofother.pos.x - pFL->points.at(0).pos.x);
					bool bCloseFromBack = false;
					if((info.bAfter == true && back_distance < 15.0) || info.bAfter == false)
						bCloseFromBack = true;


					if(fabs(info.perp_distance) < 2 && fabs(info.perp_distance) < closest_d && fabs(info.angle_diff) < max_angle_diff && info.bBefore == false && bCloseFromBack)
					{
						closest_d = fabs(info.perp_distance);
						pL = &sp_lanes.at(l);
						closest_info = info;
						closest_index = l;
					}
				}
			}

			if(pL != nullptr && pFL != nullptr)
			{
				if(closest_info.iFront == pL->points.size()-1)
				{
					pL->toIds.push_back(pFL->id);
					pL->points.at(closest_info.iFront).toIds.push_back(pFL->points.at(0).id);

					pFL->points.at(0).fromIds.push_back(pL->points.at(closest_info.iFront).id);
					pFL->fromIds.push_back(pL->id);
					bAtleastOneChange = true;
//					if(DEBUG_MAP_PARSING)
//					{
//						cout << "Closest Next Lane For: " << pFL->id << " , Is:" << pL->id << ", Distance=" << fabs(closest_info.perp_distance) << ", Size: " << pL->points.size() << ", back_index: " << closest_info.iBack <<", front_index: " << closest_info.iFront << ", Direct: " << closest_info.angle_diff << endl;
//						cout << "Don't Split , Perfect !" << endl;
//					}
				}
				else
				{
					 // split from previous point
//					if(DEBUG_MAP_PARSING)
//						cout << "Closest Next Lane For: " << pFL->id << " , Is:" << pL->id << ", Distance=" << fabs(closest_info.perp_distance) << ", Size: " << pL->points.size() << ", back_index: " << closest_info.iBack <<", front_index: " << closest_info.iFront << ", Direct: " << closest_info.angle_diff << endl;

					Lane front_half, back_half;
					front_half.points.insert(front_half.points.begin(), pL->points.begin()+closest_info.iFront, pL->points.end());
					front_half.toIds = pL->toIds;
					front_half.fromIds.push_back(pL->id);
					front_half.id = front_half.points.at(0).originalMapID;
					front_half.areaId = pL->areaId;
					front_half.dir = pL->dir;
					front_half.num = pL->num;
					front_half.roadId = pL->roadId;
					front_half.speed = pL->speed;
					front_half.type = pL->type;
					front_half.width = pL->width;

					back_half.points.insert(back_half.points.begin(), pL->points.begin(), pL->points.begin()+closest_info.iFront);
					back_half.toIds.clear();
					back_half.toIds.push_back(front_half.id);
					back_half.toIds.push_back(pFL->id);
					back_half.fromIds = pL->fromIds;
					back_half.id = pL->id;
					back_half.areaId = pL->areaId;
					back_half.dir = pL->dir;
					back_half.num = pL->num;
					back_half.roadId = pL->roadId;
					back_half.speed = pL->speed;
					back_half.type = pL->type;
					back_half.width = pL->width;

					WayPoint* last_from_back =  &back_half.points.at(back_half.points.size()-1);
					WayPoint* first_from_front =  &pFL->points.at(0);

					last_from_back->toIds.push_back(first_from_front->id);
					first_from_front->fromIds.push_back(last_from_back->id);

					if(front_half.points.size() > 1 && back_half.points.size() > 1)
					{
//						if(DEBUG_MAP_PARSING)
//							cout << "Split this one Nicely! first_half_size: " << front_half.points.size() << ", second_hald_size: " << back_half.points.size() << endl;

						pFL->fromIds.push_back(back_half.id);

						if(closest_index >= 0)
							sp_lanes.erase(sp_lanes.begin()+closest_index);
						else
							std::cout << "## Alert Alert Alert !!!! " << std::endl;

						// add perp point to lane points
						MappingHelpers::InsertWayPointToBackOfLane(closest_info.perp_point, front_half, RoadNetwork::g_max_point_id);
						MappingHelpers::InsertWayPointToFrontOfLane(closest_info.perp_point, back_half, RoadNetwork::g_max_point_id);

						sp_lanes.push_back(front_half);
						sp_lanes.push_back(back_half);
						bAtleastOneChange = true;
					}
					else
					{
//						if(DEBUG_MAP_PARSING)
//							cout << "=> Can't Split this one :( !" << endl;
					}
				}
			}
			else
			{
//				if(DEBUG_MAP_PARSING)
//					cout << "=> Can't find Before Lanes For:  " << lanes.at(il).id  << endl;
			}
		}
	}

	lanes = sp_lanes;

	//Find to lanes

	for(unsigned int il=0; il < lanes.size(); il ++)
	{
		if(lanes.at(il).toIds.size() == 0)
		{
			double closest_d = DBL_MAX;
			Lane* pL = nullptr;
			Lane* pBL = nullptr;
			for(int l=0; l < sp_lanes.size(); l ++)
			{
				if(lanes.at(il).id == sp_lanes.at(l).id)
				{
					pBL = &sp_lanes.at(l);
					break;
				}
			}

			PlannerHNS::RelativeInfo closest_info;
			int closest_index = -1;
			for(int l=0; l < sp_lanes.size(); l ++)
			{
				if(pBL->id != sp_lanes.at(l).id )
				{
					PlannerHNS::RelativeInfo info;
					WayPoint firstofother = sp_lanes.at(l).points.at(0);
					WayPoint last_point = pBL->points.at(pBL->points.size()-1);
					PlanningHelpers::GetRelativeInfoLimited(sp_lanes.at(l).points, last_point, info, 0);
					double front_distance = hypot(firstofother.pos.y - last_point.pos.y, firstofother.pos.x - last_point.pos.x);
					bool bCloseFromFront = false;
					if((info.bBefore == true && front_distance < 15.0) || info.bBefore == false)
						bCloseFromFront = true;

					if(fabs(info.perp_distance) < 2 && fabs(info.perp_distance) < closest_d && fabs(info.angle_diff) < max_angle_diff && info.bAfter == false && bCloseFromFront)
					{
						closest_d = fabs(info.perp_distance);
						pL = &sp_lanes.at(l);
						closest_info = info;
						closest_index = l;
					}
				}
			}

			if(pL != nullptr && pBL != nullptr)
			{
				if(closest_info.iFront == 0)
				{
					pL->fromIds.push_back(pBL->id);
					pL->points.at(closest_info.iFront).fromIds.push_back(pBL->points.at(pBL->points.size()-1).id);

					pBL->points.at(pBL->points.size()-1).toIds.push_back(pL->points.at(closest_info.iFront).id);
					pBL->toIds.push_back(pL->id);

					bAtleastOneChange = true;
//					if(DEBUG_MAP_PARSING)
//					{
//						cout << "Closest Back Lane For: " << pBL->id << " , Is:" << pL->id << ", Distance=" << fabs(closest_info.perp_distance) << ", Size: " << pL->points.size() << ", back_index: " << closest_info.iBack <<", front_index: " << closest_info.iFront << ", Direct: " << closest_info.angle_diff << endl;
//						cout << "Don't Split , Perfect !" << endl;
//					}
				}
				else
				{
					 // split from previous point
//					if(DEBUG_MAP_PARSING)
//						cout << "Closest Back Lane For: " << pBL->id << " , Is:" << pL->id << ", Distance=" << fabs(closest_info.perp_distance) << ", Size: " << pL->points.size() << ", back_index: " << closest_info.iBack <<", front_index: " << closest_info.iFront << ", Direct: " << closest_info.angle_diff << endl;

					Lane front_half, back_half;
					front_half.points.insert(front_half.points.begin(), pL->points.begin()+closest_info.iFront, pL->points.end());
					front_half.toIds = pL->toIds;
					front_half.fromIds.push_back(pL->id);
					front_half.fromIds.push_back(pBL->id);
					front_half.id = front_half.points.at(0).originalMapID;
					front_half.areaId = pL->areaId;
					front_half.dir = pL->dir;
					front_half.num = pL->num;
					front_half.roadId = pL->roadId;
					front_half.speed = pL->speed;
					front_half.type = pL->type;
					front_half.width = pL->width;

					back_half.points.insert(back_half.points.begin(), pL->points.begin(), pL->points.begin()+closest_info.iFront);
					back_half.toIds.push_back(front_half.id);
					back_half.fromIds = pL->fromIds;
					back_half.id = pL->id;
					back_half.areaId = pL->areaId;
					back_half.dir = pL->dir;
					back_half.num = pL->num;
					back_half.roadId = pL->roadId;
					back_half.speed = pL->speed;
					back_half.type = pL->type;
					back_half.width = pL->width;

					WayPoint* first_from_next =  &front_half.points.at(0);
					WayPoint* last_from_front =  &pBL->points.at(pBL->points.size()-1);

					first_from_next->fromIds.push_back(last_from_front->id);
					last_from_front->toIds.push_back(first_from_next->id);

					if(front_half.points.size() > 1 && back_half.points.size() > 1)
					{
//						if(DEBUG_MAP_PARSING)
//							cout << "Split this one Nicely! first_half_size: " << front_half.points.size() << ", second_hald_size: " << back_half.points.size() << endl;

						pBL->toIds.push_back(front_half.id);

						if(closest_index >= 0)
							sp_lanes.erase(sp_lanes.begin()+closest_index);
						else
							std::cout << "## Alert Alert Alert !!!! " << std::endl;

						// add perp point to lane points
						MappingHelpers::InsertWayPointToBackOfLane(closest_info.perp_point, front_half, RoadNetwork::g_max_point_id);
						MappingHelpers::InsertWayPointToFrontOfLane(closest_info.perp_point, back_half, RoadNetwork::g_max_point_id);

						sp_lanes.push_back(front_half);
						sp_lanes.push_back(back_half);
						bAtleastOneChange = true;
					}
					else
					{
//						if(DEBUG_MAP_PARSING)
//							cout << "=> Can't Split this one :( !" << endl;
					}
				}
			}
			else
			{
//				if(DEBUG_MAP_PARSING)
//					cout << "=> Can't find After Lanes For:  " << lanes.at(il).id  << endl;
			}
		}
	}

	lanes = sp_lanes;
}

void VectorMapLoader::AssignActionCostToLane(Lane* pL, ACTION_TYPE action, double cost)
{
  for(unsigned int j = 0 ; j < pL->points.size(); j++)
  {
      pL->points.at(j).actionCost.clear();
      pL->points.at(j).actionCost.push_back(std::make_pair(action, cost));
  }
}

void VectorMapLoader::ConstructRoadNetworkFromROSMessageVer0(UtilityHNS::MapRaw& mapRaw, const PlannerHNS::WayPoint& origin, PlannerHNS::RoadNetwork& map)
{

}

} /* namespace PlannerHNS */
