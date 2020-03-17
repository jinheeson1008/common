/*
 * VectorMapLoader.h
 *
 *  Created on: Mar 16, 2020
 *      Author: hatem
 */

/// \file VectorMapLoader.h
/// \brief Functions for Loading AISAN vector maps and initialize RoadNetwork map for OpenPlanner.
/// 		It could load from folder consists of .csv files, or from messages published by vector map loader in autoware
/// \author Hatem Darweesh
/// \date Mar 16, 2020

#ifndef VECTORMAPLOADER_H_
#define VECTORMAPLOADER_H_

#include <op_planner/MappingHelpers.h>
#include <op_utility/DataRW.h>

namespace PlannerHNS {

class VectorMapLoader {
public:
	VectorMapLoader(int map_version = 1, bool enable_lane_change = false, bool load_curb = false, bool load_lines = false, bool load_wayarea = false);
	virtual ~VectorMapLoader();

	/**
	 * @brief Sometimes additional origin needed to align the map, currently default origin is zero
	 * @param origin
	 */
	void SetAdditionalOrigin(const PlannerHNS::WayPoint& origin);

	/**
	 * @brief Read .csv based vector map files format. It is main format used by Autoware framework, developed by AISAN Technologies.
	 * 			Only .csv files matching predefined item names will be loaded.
	 * @param vector map folder name
	 * @param map, OpenPlanner RoadNetwork map
	 */
	void LoadFromFile(const std::string& vectorMapFolder, PlannerHNS::RoadNetwork& map);

	/**
	 * @brief Convert vector map from data items raw data into OpenPlanner RoadNetwork format
	 * @param mapRaw, contains a list of all vector map items, which representing each .csv file.
	 * @param map, OpenPlanner RoadNetwork map
	 */
	void LoadFromData(UtilityHNS::MapRaw& mapRaw, PlannerHNS::RoadNetwork& map);

private:
	int _map_version;
	bool _find_parallel_lanes;
	bool _load_curbs;
	bool _load_lines;
	bool _load_wayareas;
	PlannerHNS::WayPoint _origin;

	void ConstructRoadNetworkFromROSMessage(UtilityHNS::MapRaw& mapRaw, const PlannerHNS::WayPoint& origin, PlannerHNS::RoadNetwork& map);
	void ConstructRoadNetworkFromROSMessageVer0(UtilityHNS::MapRaw& mapRaw, const PlannerHNS::WayPoint& origin, PlannerHNS::RoadNetwork& map);

	void CreateLanes(UtilityHNS::AisanLanesFileReader* pLaneData, UtilityHNS::AisanPointsFileReader* pPointsData,
			UtilityHNS::AisanNodesFileReader* pNodesData, std::vector<PlannerHNS::Lane>& out_lanes);
	void ConnectLanes(UtilityHNS::AisanLanesFileReader* pLaneData, std::vector<PlannerHNS::Lane>& lanes);
	void GetLanesStartPoints(UtilityHNS::AisanLanesFileReader* pLaneData, std::vector<int>& m_LanesStartIds);
	void GetLanePoints(UtilityHNS::AisanLanesFileReader* pLaneData, UtilityHNS::AisanPointsFileReader* pPointsData,
				UtilityHNS::AisanNodesFileReader* pNodesData, int lnID, PlannerHNS::Lane& out_lane);
	bool IsStartLanePoint(UtilityHNS::AisanLanesFileReader* pLaneData, UtilityHNS::AisanLanesFileReader::AisanLane* pL);
	bool IsEndLanePoint(UtilityHNS::AisanLanesFileReader* pLaneData, UtilityHNS::AisanLanesFileReader::AisanLane* pL);
	bool GetPointFromDataList(UtilityHNS::AisanPointsFileReader* pPointsData,const int& pid, WayPoint& out_wp);
	int GetBeginPointIdFromLaneNo(UtilityHNS::AisanLanesFileReader* pLaneData, UtilityHNS::AisanPointsFileReader* pPointsData,
				UtilityHNS::AisanNodesFileReader* pNodesData, const int& LnID);
	int GetEndPointIdFromLaneNo(UtilityHNS::AisanLanesFileReader* pLaneData, UtilityHNS::AisanPointsFileReader* pPointsData,
				UtilityHNS::AisanNodesFileReader* pNodesData,const int& LnID);
	bool IsPointExist(const WayPoint& p, const std::vector<PlannerHNS::WayPoint>& points);
	void FixRedundantPointsLanes(std::vector<Lane>& lanes);
	void FixTwoPointsLanes(std::vector<Lane>& lanes);
	void FixTwoPointsLane(Lane& lanes);
	void FixUnconnectedLanes(std::vector<Lane>& lanes, const int& max_angle_diff = 90);
	void AssignActionCostToLane(Lane* pL, ACTION_TYPE action, double cost);
};

} /* namespace PlannerHNS */

#endif /* VECTORMAPLOADER_H_ */
