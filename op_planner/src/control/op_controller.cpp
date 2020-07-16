
/// \file op_controller.cpp
/// \brief PID based trajectory follower and velocity controller
/// \author Hatem Darweesh
/// \date July 04, 2020

#include "op_planner/control/op_controller.h"
#include "op_planner/PlanningHelpers.h"
#include <cmath>
#include <stdlib.h>
#include <iostream>

namespace PlannerHNS
{

MotionControl::MotionControl()
{
	m_TargetAngle = 0;
	m_TargetSpeed = 0;
	m_PrevAngleError = 0;
	m_PrevSpeedError = 0;
	m_PrevSpeed = 0;
	m_iNextTest = 0;
	m_bCalibrationMode = false;
	m_bEnableLog = false;
	m_FollowingDistance = 0;
	m_LateralError 		= 0;
	m_PrevDesiredTorque	= 0;
	m_PrevDesiredAccelStroke = 0;
	m_PrevDesiredBrakeStroke = 0;
	m_FollowAcceleration= 0;
	m_iPrevWayPoint 	= -1;
	m_iCalculatedIndex = 0;
	m_CruseSpeedRange = -2;
	m_AccelerationSum = 0;
	m_nAccelerations = 0;
	m_AverageAcceleration = 0;

	UtilityHNS::UtilityH::GetTickCount(m_SteerDelayTimer);
	UtilityHNS::UtilityH::GetTickCount(m_VelocityDelayTimer);
	ResetLogTime(0,0);
}

void MotionControl::Init(const ControllerParams& params, const CAR_BASIC_INFO& vehicleInfo, bool bEnableLogs, bool bCalibration)
{
	m_bEnableLog = bEnableLogs;
	m_bCalibrationMode = bCalibration;
	if(m_bCalibrationMode)
	{
		InitCalibration();
	}

	m_Params = params;
	m_VehicleInfo = vehicleInfo;

	m_lowpassSteer.Init(m_Params.ControlFrequency, m_Params.LowpassSteerCutoff);
	m_pidSteer.Init(m_Params.Steering_Gain.kP, m_Params.Steering_Gain.kI, m_Params.Steering_Gain.kD); // for 3 m/s
	m_pidSteer.Setlimit(m_VehicleInfo.max_steer_torque, m_VehicleInfo.min_steer_torque);
	m_pidAccelBrake.Init(m_Params.Accel_Gain.kP, m_Params.Accel_Gain.kI, m_Params.Accel_Gain.kD);
	m_pidAccelBrake.Setlimit(m_VehicleInfo.max_accel_value, -m_VehicleInfo.max_brake_value);

	m_pidAccel.Init(m_Params.Accel_Gain.kP, m_Params.Accel_Gain.kI, m_Params.Accel_Gain.kD);
	m_pidAccel.Setlimit(m_VehicleInfo.max_accel_value, 0);
	m_pidBrake.Init(m_Params.Brake_Gain.kP, m_Params.Brake_Gain.kI, m_Params.Brake_Gain.kD);
	m_pidBrake.Setlimit(m_VehicleInfo.max_brake_value, 0);
}

MotionControl::~MotionControl()
{
	if(m_bEnableLog)
	{
		std::ostringstream fileName;
		if(m_ExperimentFolderName.size() == 0)
			fileName << UtilityHNS::UtilityH::GetHomeDirectory()+UtilityHNS::DataRW::LoggingMainfolderName + UtilityHNS::DataRW::ControlLogFolderName;
		else
			fileName << UtilityHNS::UtilityH::GetHomeDirectory()+UtilityHNS::DataRW::LoggingMainfolderName + UtilityHNS::DataRW::ExperimentsFolderName + m_ExperimentFolderName + UtilityHNS::DataRW::ControlLogFolderName;

		UtilityHNS::DataRW::WriteLogData(fileName.str(), "ControlLog",
				"dt,t,X,Y,"
				"heading,target_angle,angle_err,torque,"
				"state,stop_distance,acceleration,"
				"speed,target_speed,speed_err,accel_stroke,"
				"brake_stroke,lateral_error,iIndex,pathSize,avg_accel,total_accel",
				m_LogData);

		UtilityHNS::DataRW::WriteLogData(fileName.str(), "SteeringCalibrationLog",
				"time, reset, start A, end A, desired A, dt, vel", m_SteerCalibrationData);

		UtilityHNS::DataRW::WriteLogData(fileName.str(), "VelocityCalibrationLog",
				"time, reset, start V, end V, desired V, dt, steering", m_VelocityCalibrationData);

		UtilityHNS::DataRW::WriteLogData(fileName.str(), "SteeringPIDLog",m_pidSteer.ToStringHeader(), m_LogSteerPIDData );
		UtilityHNS::DataRW::WriteLogData(fileName.str(), "AccelPIDLog",m_pidAccel.ToStringHeader(), m_LogAccelerationPIDData );
		UtilityHNS::DataRW::WriteLogData(fileName.str(), "BrakePIDLog",m_pidBrake.ToStringHeader(), m_LogBrakingPIDData );
		UtilityHNS::DataRW::WriteLogData(fileName.str(), "LinearPIDLog",m_pidAccelBrake.ToStringHeader(), m_LogLinearPIDData );
	}
}

void MotionControl::ResetLogTime(const double& v0, const double& v1)
{
	double time_total = UtilityHNS::UtilityH::GetTimeDiffNow(m_LogTimer);
	if(time_total != 0)
	{
		m_TotalAcceleration = (v1 - v0) / time_total;
	}

	if(m_nAccelerations != 0)
	{
		m_AverageAcceleration = m_AccelerationSum / m_nAccelerations;
	}

	UtilityHNS::UtilityH::GetTickCount(m_LogTimer);
}

void MotionControl::SetCruiseSpeedRange(const double& speed_range)
{
	m_CruseSpeedRange = speed_range;
}

void MotionControl::UpdateCurrentPath(const std::vector<PlannerHNS::WayPoint>& path)
{
	m_Path = path;
}

bool MotionControl::FindNextWayPoint(const std::vector<PlannerHNS::WayPoint>& path, const PlannerHNS::WayPoint& state,
		const double& velocity, PlannerHNS::WayPoint& pursuite_point, PlannerHNS::WayPoint& prep,
		double& lateral_err, double& follow_distance)
{
	if(path.size()==0) return false;

	follow_distance = fabs(velocity*1.5);

	if(follow_distance < m_Params.minPursuiteDistance)
	{
		follow_distance = m_Params.minPursuiteDistance;
	}

	RelativeInfo info;
	PlanningHelpers::GetRelativeInfo(path, state, info);
	unsigned int dummy_index = 0;
	pursuite_point = PlanningHelpers::GetFollowPointOnTrajectory(path, info, follow_distance, dummy_index);
	prep = info.perp_point;
	lateral_err = info.perp_distance;
	m_iPrevWayPoint = info.iFront;

	return true;
}

int MotionControl::SteerControllerUpdate(const double& dt, const PlannerHNS::WayPoint& CurrPose, const PlannerHNS::WayPoint& TargetPose,
		const PlannerHNS::VehicleState& CurrStatus, const PlannerHNS::BehaviorState& CurrBehavior,
		const double& lateralErr, double& desiredSteerTorque)
{
	int ret = 1;

	if(CurrBehavior.state != INITIAL_STATE && CurrBehavior.state != FINISH_STATE)
	{
		ret = SteerControllerPart(dt, CurrPose, TargetPose, lateralErr, desiredSteerTorque);
	}
	else
	{
		desiredSteerTorque = 0;
	}

	if(ret < 0)
	{
		desiredSteerTorque = m_PrevDesiredTorque;
	}
	else
	{
		m_PrevDesiredTorque = desiredSteerTorque;
	}

	return ret;
}

int MotionControl::SteerControllerPart(const double& dt, const PlannerHNS::WayPoint& state, const PlannerHNS::WayPoint& way_point,
		const double& lateral_error, double& steerd)
{
	double current_a = UtilityHNS::UtilityH::SplitPositiveAngle(state.pos.a);
	double target_a = atan2(way_point.pos.y - state.pos.y, way_point.pos.x - state.pos.x);
	double e =  UtilityHNS::UtilityH::SplitPositiveAngle(current_a - target_a);

	if((e > 0 && m_PrevAngleError < 0) || (e < 0 && m_PrevAngleError > 0))
	{
		m_pidSteer.ResetI();
	}

	//ToDo Activate this later, check first if this condition ever happens
//	if(e > M_PI_2 || e < -M_PI_2)
//	{
//		return -1;
//	}

	//TODO use lateral error instead of angle error

	double before_lowpass = m_pidSteer.getTimeDependentPID(e, dt);


	steerd = before_lowpass;
	m_TargetAngle = target_a;
	m_PrevAngleError = e;

	if(m_bEnableLog)
	{
		m_LogSteerPIDData.push_back(m_pidSteer.ToString());
	}
	return 1;
}

void MotionControl::PredictMotion(double& x, double &y, double& heading, double steering, double velocity, double wheelbase, double time_elapsed)
{
	x += velocity * time_elapsed *  cos(heading);
	y += velocity * time_elapsed *  sin(heading);
	heading = heading + ((velocity*time_elapsed*tan(steering))  / (wheelbase) );
}

double MotionControl::CalculateVelocityDesired(const double& dt, const PlannerHNS::VehicleState& CurrStatus,
			const PlannerHNS::BehaviorState& CurrBehavior)
{
	double desired_velocity = CurrBehavior.maxVelocity;
	if(desired_velocity > m_VehicleInfo.max_speed_forward)
	{
		desired_velocity = m_VehicleInfo.max_speed_forward;
	}

	//calculate acceleration
	double dv = CurrStatus.speed - m_PrevSpeed;
	if(dv != 0 && dt != 0)
	{
		m_InstantAcceleration = dv/dt;
	}

	m_AccelerationSum += m_InstantAcceleration;
	m_nAccelerations++;

	if(CurrBehavior.state == TRAFFIC_LIGHT_STOP_STATE || CurrBehavior.state == STOP_SIGN_STOP_STATE || CurrBehavior.state == STOPPING_STATE)
	{
		if(CurrBehavior.stopDistance > 0)
		{
			double deceleration_critical = (-CurrStatus.speed*CurrStatus.speed)/(2.0*CurrBehavior.stopDistance);
			if(deceleration_critical > m_VehicleInfo.max_deceleration)
			{
				deceleration_critical = m_VehicleInfo.max_deceleration;
			}

			desired_velocity = CurrStatus.speed + (deceleration_critical * dt);

			std::cout << "Deceleration : " <<  deceleration_critical << std::endl;
		}
		else
		{
			desired_velocity = 0;
		}
	}
	else if(CurrBehavior.state == FORWARD_STATE || CurrBehavior.state == OBSTACLE_AVOIDANCE_STATE )
	{
//		double v_desired = CurrStatus.speed + (m_VehicleInfo.max_acceleration * dt);
	}
	else if(CurrBehavior.state == FINISH_STATE)
	{
		desired_velocity = 0;
	}
	else if(CurrBehavior.state == FOLLOW_STATE)
	{
	}
	else
	{
	}

	return desired_velocity;
}

int MotionControl::VeclocityControllerUpdateOnePID(const double& dt, const PlannerHNS::VehicleState& CurrStatus,
		const PlannerHNS::BehaviorState& CurrBehavior, double& desiredAccel, double& desiredBrake, PlannerHNS::SHIFT_POS& desiredShift)
{

	double desired_velocity = CalculateVelocityDesired(dt, CurrStatus, CurrBehavior);

	desiredShift = PlannerHNS::SHIFT_POS_DD;
	double e = (desired_velocity - CurrStatus.speed);

//	if((e > 0 && m_PrevSpeedError < 0) || (e < 0 && m_PrevSpeedError > 0))
//	{
//		m_pidAccelBrake.ResetI();
//	}

	double desiredStroke = m_pidAccelBrake.getTimeDependentPID(e, dt);

	if(desiredStroke > 0)
	{
		desiredAccel = desiredStroke;
		desiredBrake = 0;
	}
	else
	{
		desiredBrake = -desiredStroke;
		desiredAccel = 0;
	}

	if(m_bEnableLog)
	{
		m_LogLinearPIDData.push_back(m_pidAccelBrake.ToString());
	}

	m_PrevDesiredBrakeStroke = desiredBrake;
	m_PrevDesiredAccelStroke = desiredAccel;
	m_TargetSpeed = desired_velocity;
	m_PrevSpeedError = e;

	return 1;
}

int MotionControl::VeclocityControllerUpdateTwoPID(const double& dt, const PlannerHNS::VehicleState& CurrStatus,
		const PlannerHNS::BehaviorState& CurrBehavior, double& desiredAccel, double& desiredBrake, PlannerHNS::SHIFT_POS& desiredShift)
{

	double desired_velocity = CalculateVelocityDesired(dt, CurrStatus, CurrBehavior);
	double deceleration_critical = (desired_velocity - CurrStatus.speed) / dt;

	desiredShift = PlannerHNS::SHIFT_POS_DD;
	double e = (desired_velocity - CurrStatus.speed);



//	if((e > 0 && m_PrevSpeedError < 0) || (e < 0 && m_PrevSpeedError > 0))
//	{
//		m_pidAccel.ResetI();
//		m_pidBrake.ResetI();
//	}


//	if( e <= 0.25 && e >= -0.25) //cruise
//	{
//		desiredAccel = m_PrevDesiredAccelStroke;
//		desiredBrake = m_PrevDesiredBrakeStroke;
//		m_pidBrake.ResetI();
//		m_pidBrake.ResetD();
//	}
	if(desired_velocity > 0 && deceleration_critical > -1.0) //accelerate , cruise, small decelerate
	{
		m_pidBrake.ResetI();
		m_pidBrake.ResetD();
		desiredAccel = m_pidAccel.getTimeDependentPID(e, dt);
		desiredBrake = 0;
	}
	else if(deceleration_critical > -2.5) // use engine brake
	{
		desiredAccel = 0;
		desiredBrake = 0;
		m_pidAccel.ResetI();
		m_pidAccel.ResetD();
	}
	else  // braking
	{
		m_pidAccel.ResetI();
		m_pidAccel.ResetD();
		desiredBrake = m_pidBrake.getTimeDependentPID(-e, dt);
		desiredAccel = 0;
	}

	m_PrevDesiredAccelStroke = desiredAccel;
	m_PrevDesiredBrakeStroke = desiredBrake;
	m_TargetSpeed = desired_velocity;
	m_PrevSpeedError = e;

	if(m_bEnableLog)
	{
		m_LogAccelerationPIDData.push_back(m_pidAccel.ToString());
		m_LogBrakingPIDData.push_back(m_pidBrake.ToString());
	}

	return 1;
}

PlannerHNS::ExtendedVehicleState MotionControl::DoOneStep(const double& dt, const PlannerHNS::BehaviorState& behavior,
		const std::vector<PlannerHNS::WayPoint>& path, const PlannerHNS::WayPoint& currPose,
		const PlannerHNS::VehicleState& vehicleState, const bool& bNewTrajectory)
{
	if(bNewTrajectory && path.size() > 0)
	{
		UpdateCurrentPath(path);
		m_iPrevWayPoint = -1;
	}

	PlannerHNS::ExtendedVehicleState desiredState;

	if(m_bCalibrationMode)
	{
		CalibrationStep(dt, vehicleState, desiredState.steer, desiredState.speed);
		desiredState.shift = PlannerHNS::SHIFT_POS_DD;
	}
	else if(m_Path.size()>0 && behavior.state != INITIAL_STATE )
	{
		FindNextWayPoint(m_Path, currPose, vehicleState.speed, m_FollowMePoint, m_PerpendicularPoint, m_LateralError, m_FollowingDistance);
		VeclocityControllerUpdateTwoPID(dt, vehicleState, behavior, desiredState.accel_stroke, desiredState.brake_stroke, desiredState.shift);
		SteerControllerUpdate(dt, currPose, m_FollowMePoint, vehicleState, behavior, m_LateralError, desiredState.steer_torque);
	}
	else
	{
		desiredState.steer = 0;
		desiredState.speed = 0;
		desiredState.shift = PlannerHNS::SHIFT_POS_DD;
		desiredState.steer_torque = 0;
		desiredState.accel_stroke = 0;
		desiredState.brake_stroke = 0;
		std::cout << "$$$$$ Error, Very Dangerous, Following No Path !!." << std::endl;
	}

	//std::cout << "Planner: State: " << behavior.state << ", stop_distance: " << behavior.stopDistance << ", followD: " << behavior.followDistance << std::endl;

	if(m_bEnableLog)
	{
		timespec t;
		UtilityHNS::UtilityH::GetTickCount(t);
		double time_total = UtilityHNS::UtilityH::GetTimeDiffNow(m_LogTimer);
		std::ostringstream time_str;
		time_str.precision(4);
		time_str << time_total;
		std::ostringstream dataLine;
		dataLine << dt << "," << time_str.str() << "," << currPose.pos.x << "," << currPose.pos.y << "," <<
				currPose.pos.a << "," << m_TargetAngle << "," << m_PrevAngleError << "," << desiredState.steer_torque << "," <<
				behavior.state << "," << behavior.stopDistance << "," << m_InstantAcceleration << "," <<
				vehicleState.speed << "," << m_TargetSpeed << "," << m_PrevSpeedError << "," <<  desiredState.accel_stroke << "," <<
				desiredState.brake_stroke <<  "," << m_LateralError << "," << m_iPrevWayPoint << "," << m_Path.size() << "," << m_AverageAcceleration <<"," << m_TotalAcceleration << ",";
		m_LogData.push_back(dataLine.str());
		LogCalibrationData(vehicleState, desiredState);
	}

	m_PrevSpeed = vehicleState.speed;

	return desiredState;
}

void MotionControl::CalibrationStep(const double& dt, const PlannerHNS::VehicleState& CurrStatus, double& desiredSteer, double& desiredVelocity)
{
	if(m_iNextTest >= (int)m_CalibrationRunList.size()-1)
	{
		desiredSteer = 0;
		desiredVelocity = 0;
		return;
	}

	if(fabs(CurrStatus.speed - m_CalibrationRunList.at(m_iNextTest).first)*3.6 <= 1
			&& fabs(CurrStatus.steer - m_CalibrationRunList.at(m_iNextTest).second)*RAD2DEG <=0.5)
	{
		m_iNextTest++;
	}

	desiredVelocity = m_CalibrationRunList.at(m_iNextTest).first;
	desiredSteer = m_CalibrationRunList.at(m_iNextTest).second;

	std::cout << "i:" << m_iNextTest << ", desVel:" << desiredVelocity << ", CurVel:" << CurrStatus.speed
			<< ", desStr:" << desiredSteer << ", CurrStr:" << CurrStatus.steer << std::endl;

//	double e = targetSpeed - CurrStatus.speed;
//	if(e >= 0)
//		desiredVelocity = (m_VehicleInfo.max_acceleration * dt) + CurrStatus.speed;
//	else
//		desiredVelocity = (m_VehicleInfo.max_deceleration * dt) + CurrStatus.speed;
}

void MotionControl::LogCalibrationData(const PlannerHNS::VehicleState& currState,const PlannerHNS::VehicleState& desiredState)
{
	int startAngle=0, finishAngle=0, originalTargetAngle=0, currVelocity = 0;
	double t_FromStartToFinish_a = 0;
	bool bAngleReset = false;
	int startV=0, finishV=0, originalTargetV=0, currSteering = 0;
	double t_FromStartToFinish_v = 0;
	bool bVelocityReset = false;

	//1- decide reset
	if((int)(m_prevDesiredState_steer.steer*RAD2DEG) != (int)(desiredState.steer*RAD2DEG))
		bAngleReset = true;

	if((int)(m_prevDesiredState_vel.speed*3.6) != (int)(desiredState.speed*3.6))
		bVelocityReset = true;

	//2- calculate time and log
	if(bAngleReset)
	{
		startAngle = m_prevCurrState_steer.steer*RAD2DEG;
		finishAngle = currState.steer*RAD2DEG;
		originalTargetAngle = m_prevDesiredState_steer.steer*RAD2DEG;
		t_FromStartToFinish_a = UtilityHNS::UtilityH::GetTimeDiffNow(m_SteerDelayTimer);
		currVelocity = currState.speed*3.6;
		UtilityHNS::UtilityH::GetTickCount(m_SteerDelayTimer);

		std::ostringstream dataLine;
		dataLine << UtilityHNS::UtilityH::GetLongTime(m_SteerDelayTimer) << ","
				<< bAngleReset << ","
				<< startAngle << ","
				<< finishAngle << ","
				<< originalTargetAngle << ","
				<< t_FromStartToFinish_a << ","
				<< currVelocity << ",";

		m_SteerCalibrationData.push_back(dataLine.str());

		if(bAngleReset)
		{
			bAngleReset = false;
			m_prevCurrState_steer = currState;
			m_prevDesiredState_steer = desiredState;
		}
	}

	if(bVelocityReset)
	{
		startV = m_prevCurrState_vel.speed*3.6;
		finishV = currState.speed*3.6;
		originalTargetV = m_prevDesiredState_vel.speed*3.6;
		t_FromStartToFinish_v = UtilityHNS::UtilityH::GetTimeDiffNow(m_VelocityDelayTimer);
		currSteering = currState.steer*RAD2DEG;
		UtilityHNS::UtilityH::GetTickCount(m_VelocityDelayTimer);

		std::ostringstream dataLine;
		dataLine << UtilityHNS::UtilityH::GetLongTime(m_VelocityDelayTimer) << ","
				<< bVelocityReset << ","
				<< startV << ","
				<< finishV << ","
				<< originalTargetV << ","
				<< t_FromStartToFinish_v << ","
				<< currSteering << ",";

		m_VelocityCalibrationData.push_back(dataLine.str());

		if(bVelocityReset)
		{
			bVelocityReset = false;
			m_prevCurrState_vel = currState;
			m_prevDesiredState_vel = desiredState;
		}
	}
}

void MotionControl::CoordinateAscent(double tolerance, PID_CONST& pOut)
{

//   double p[3] = {0,0,0};
//   double pd[3] = {1,1,1};
//
//   int N = 100;
//   float change_factor = 0.01;
//   CAR_BASIC_INFO* inf =  m_Config->GetCarBasicInfo();
//   inf->velocity_pid.kP=p[0];
//   inf->velocity_pid.kI=p[1];
//   inf->velocity_pid.kD=p[2];
//   double err = 0;
//   double best_error=0, e=0;
//
//	pCurrBehavior->Start();
//	for(int j=0;j<N*2; j++)
//	{
//		StepMoveToObjective();
//		if(j>=N)
//			e += pCurrBehavior->m_SpeedErr*pCurrBehavior->m_SpeedErr;
//	}
//	best_error = e/(double)N;
//
//    int n = 0;
//	while( tolerance < (pd[0]+pd[1]+pd[2]))
//	{
//        for(int i=0; i< 3; i++)
//		{
//            p[i] += pd[i];
//            e=0;
//			inf->velocity_pid.kP=p[0];
//			inf->velocity_pid.kI=p[1];
//			inf->velocity_pid.kD=p[2];
//			ResetPlanner();
//			pCurrBehavior->Start();
//			for(int j=0;j<N*2; j++)
//			{
//				StepMoveToObjective(true);
//				if(j>=N)
//					e += pCurrBehavior->m_SpeedErr*pCurrBehavior->m_SpeedErr;
//			}
//
//			err = e/(double)N;
//			if( err < best_error)
//			{
//                best_error = err;
//               pd[i] *= change_factor;
//			}
//            else
//			{
//                p[i] -= (2 * pd[i]);
//
//				e=0;
//				inf->velocity_pid.kP=p[0];
//				inf->velocity_pid.kI=p[1];
//				inf->velocity_pid.kD=p[2];
//
//				ResetPlanner();
//				pCurrBehavior->Start();
//				for(int j=0;j<N*2; j++)
//				{
//					StepMoveToObjective(true);
//					if(j>=N)
//						e += pCurrBehavior->m_SpeedErr*pCurrBehavior->m_SpeedErr;
//				}
//                err = e/(double)N;
//                if (err < best_error)
//				{
//                    best_error = err;
//                    pd[i] *= change_factor;
//				}
//                else
//				{
//                    p[i] += pd[i];
//                    pd[i] *= (1-change_factor);
//				}
//			}
//		}
//		n+=1;
//
//
//	}
//	pOut.kP = p[0];
//	pOut.kI = p[1];
//	pOut.kD = p[2];

}

void MotionControl::InitCalibration()
{
	m_CalibrationRunList.push_back(std::make_pair(0,0));
	m_CalibrationRunList.push_back(std::make_pair(0,m_VehicleInfo.max_wheel_angle/4.0));
	m_CalibrationRunList.push_back(std::make_pair(0,0.0));
	m_CalibrationRunList.push_back(std::make_pair(0,-m_VehicleInfo.max_wheel_angle/4.0));
	m_CalibrationRunList.push_back(std::make_pair(0,m_VehicleInfo.max_wheel_angle/2.0));
	m_CalibrationRunList.push_back(std::make_pair(0,0.0));
	m_CalibrationRunList.push_back(std::make_pair(0,-m_VehicleInfo.max_wheel_angle/2.0));
	m_CalibrationRunList.push_back(std::make_pair(0,m_VehicleInfo.max_wheel_angle/1.5));
	m_CalibrationRunList.push_back(std::make_pair(0,0.0));
	m_CalibrationRunList.push_back(std::make_pair(0,-m_VehicleInfo.max_wheel_angle/1.5));
	m_CalibrationRunList.push_back(std::make_pair(0,m_VehicleInfo.max_wheel_angle/1.0));
	m_CalibrationRunList.push_back(std::make_pair(0,0.0));
	m_CalibrationRunList.push_back(std::make_pair(0,-m_VehicleInfo.max_wheel_angle/1.0));

	m_CalibrationRunList.push_back(std::make_pair(1,0));
	m_CalibrationRunList.push_back(std::make_pair(1,m_VehicleInfo.max_wheel_angle/4.0));
	m_CalibrationRunList.push_back(std::make_pair(1,0.0));
	m_CalibrationRunList.push_back(std::make_pair(1,-m_VehicleInfo.max_wheel_angle/4.0));
	m_CalibrationRunList.push_back(std::make_pair(1,m_VehicleInfo.max_wheel_angle/2.0));
	m_CalibrationRunList.push_back(std::make_pair(1,0.0));
	m_CalibrationRunList.push_back(std::make_pair(1,-m_VehicleInfo.max_wheel_angle/2.0));
	m_CalibrationRunList.push_back(std::make_pair(1,m_VehicleInfo.max_wheel_angle/1.5));
	m_CalibrationRunList.push_back(std::make_pair(1,0.0));
	m_CalibrationRunList.push_back(std::make_pair(1,-m_VehicleInfo.max_wheel_angle/1.5));
	m_CalibrationRunList.push_back(std::make_pair(1,m_VehicleInfo.max_wheel_angle/1.0));
	m_CalibrationRunList.push_back(std::make_pair(1,0.0));
	m_CalibrationRunList.push_back(std::make_pair(1,-m_VehicleInfo.max_wheel_angle/1.0));

	m_CalibrationRunList.push_back(std::make_pair(0,0));

	m_CalibrationRunList.push_back(std::make_pair(2,0));
	m_CalibrationRunList.push_back(std::make_pair(2,m_VehicleInfo.max_wheel_angle/4.0));
	m_CalibrationRunList.push_back(std::make_pair(2,0.0));
	m_CalibrationRunList.push_back(std::make_pair(2,-m_VehicleInfo.max_wheel_angle/4.0));
	m_CalibrationRunList.push_back(std::make_pair(2,m_VehicleInfo.max_wheel_angle/2.0));
	m_CalibrationRunList.push_back(std::make_pair(2,0.0));
	m_CalibrationRunList.push_back(std::make_pair(2,-m_VehicleInfo.max_wheel_angle/2.0));
	m_CalibrationRunList.push_back(std::make_pair(2,m_VehicleInfo.max_wheel_angle/1.5));
	m_CalibrationRunList.push_back(std::make_pair(2,0.0));
	m_CalibrationRunList.push_back(std::make_pair(2,-m_VehicleInfo.max_wheel_angle/1.5));
	m_CalibrationRunList.push_back(std::make_pair(2,m_VehicleInfo.max_wheel_angle/1.0));
	m_CalibrationRunList.push_back(std::make_pair(2,0.0));
	m_CalibrationRunList.push_back(std::make_pair(2,-m_VehicleInfo.max_wheel_angle/1.0));

	m_CalibrationRunList.push_back(std::make_pair(0,0));

	m_CalibrationRunList.push_back(std::make_pair(3,0));
	m_CalibrationRunList.push_back(std::make_pair(3,m_VehicleInfo.max_wheel_angle/4.0));
	m_CalibrationRunList.push_back(std::make_pair(3,0.0));
	m_CalibrationRunList.push_back(std::make_pair(3,-m_VehicleInfo.max_wheel_angle/4.0));
	m_CalibrationRunList.push_back(std::make_pair(3,m_VehicleInfo.max_wheel_angle/2.0));
	m_CalibrationRunList.push_back(std::make_pair(3,0.0));
	m_CalibrationRunList.push_back(std::make_pair(3,-m_VehicleInfo.max_wheel_angle/2.0));
	m_CalibrationRunList.push_back(std::make_pair(3,m_VehicleInfo.max_wheel_angle/1.5));
	m_CalibrationRunList.push_back(std::make_pair(3,0.0));
	m_CalibrationRunList.push_back(std::make_pair(3,-m_VehicleInfo.max_wheel_angle/1.5));

	m_CalibrationRunList.push_back(std::make_pair(0,0));

	m_CalibrationRunList.push_back(std::make_pair(4,0));
	m_CalibrationRunList.push_back(std::make_pair(4,m_VehicleInfo.max_wheel_angle/4.0));
	m_CalibrationRunList.push_back(std::make_pair(4,0.0));
	m_CalibrationRunList.push_back(std::make_pair(4,-m_VehicleInfo.max_wheel_angle/4.0));
	m_CalibrationRunList.push_back(std::make_pair(4,m_VehicleInfo.max_wheel_angle/2.0));
	m_CalibrationRunList.push_back(std::make_pair(4,0.0));
	m_CalibrationRunList.push_back(std::make_pair(4,-m_VehicleInfo.max_wheel_angle/2.0));
	m_CalibrationRunList.push_back(std::make_pair(4,m_VehicleInfo.max_wheel_angle/1.5));
	m_CalibrationRunList.push_back(std::make_pair(4,0.0));
	m_CalibrationRunList.push_back(std::make_pair(4,-m_VehicleInfo.max_wheel_angle/1.5));

	m_CalibrationRunList.push_back(std::make_pair(0,0));

	m_CalibrationRunList.push_back(std::make_pair(5,0));
	m_CalibrationRunList.push_back(std::make_pair(5,m_VehicleInfo.max_wheel_angle/4.0));
	m_CalibrationRunList.push_back(std::make_pair(5,0.0));
	m_CalibrationRunList.push_back(std::make_pair(5,-m_VehicleInfo.max_wheel_angle/4.0));
	m_CalibrationRunList.push_back(std::make_pair(5,m_VehicleInfo.max_wheel_angle/2.0));
	m_CalibrationRunList.push_back(std::make_pair(5,0.0));
	m_CalibrationRunList.push_back(std::make_pair(5,-m_VehicleInfo.max_wheel_angle/2.0));
	m_CalibrationRunList.push_back(std::make_pair(5,m_VehicleInfo.max_wheel_angle/1.5));
	m_CalibrationRunList.push_back(std::make_pair(5,0.0));
	m_CalibrationRunList.push_back(std::make_pair(5,-m_VehicleInfo.max_wheel_angle/1.5));

	m_CalibrationRunList.push_back(std::make_pair(0,0));

	m_CalibrationRunList.push_back(std::make_pair(6,0));
	m_CalibrationRunList.push_back(std::make_pair(6,m_VehicleInfo.max_wheel_angle/4.0));
	m_CalibrationRunList.push_back(std::make_pair(6,0.0));
	m_CalibrationRunList.push_back(std::make_pair(6,-m_VehicleInfo.max_wheel_angle/4.0));
	m_CalibrationRunList.push_back(std::make_pair(6,m_VehicleInfo.max_wheel_angle/3.0));
	m_CalibrationRunList.push_back(std::make_pair(6,0.0));
	m_CalibrationRunList.push_back(std::make_pair(6,-m_VehicleInfo.max_wheel_angle/3.0));
	m_CalibrationRunList.push_back(std::make_pair(6,m_VehicleInfo.max_wheel_angle/2.0));
	m_CalibrationRunList.push_back(std::make_pair(6,0.0));
	m_CalibrationRunList.push_back(std::make_pair(6,-m_VehicleInfo.max_wheel_angle/2.0));

	m_CalibrationRunList.push_back(std::make_pair(0,0));

	m_CalibrationRunList.push_back(std::make_pair(7,0));
	m_CalibrationRunList.push_back(std::make_pair(7,m_VehicleInfo.max_wheel_angle/4.0));
	m_CalibrationRunList.push_back(std::make_pair(7,0.0));
	m_CalibrationRunList.push_back(std::make_pair(7,-m_VehicleInfo.max_wheel_angle/4.0));
	m_CalibrationRunList.push_back(std::make_pair(7,m_VehicleInfo.max_wheel_angle/3.0));
	m_CalibrationRunList.push_back(std::make_pair(7,0.0));
	m_CalibrationRunList.push_back(std::make_pair(7,-m_VehicleInfo.max_wheel_angle/3.0));
	m_CalibrationRunList.push_back(std::make_pair(7,m_VehicleInfo.max_wheel_angle/2.0));
	m_CalibrationRunList.push_back(std::make_pair(7,0.0));
	m_CalibrationRunList.push_back(std::make_pair(7,-m_VehicleInfo.max_wheel_angle/2.0));

	m_CalibrationRunList.push_back(std::make_pair(0,0));

	m_CalibrationRunList.push_back(std::make_pair(8,0));
	m_CalibrationRunList.push_back(std::make_pair(8,m_VehicleInfo.max_wheel_angle/6.0));
	m_CalibrationRunList.push_back(std::make_pair(8,0.0));
	m_CalibrationRunList.push_back(std::make_pair(8,-m_VehicleInfo.max_wheel_angle/6.0));
	m_CalibrationRunList.push_back(std::make_pair(8,m_VehicleInfo.max_wheel_angle/4.0));
	m_CalibrationRunList.push_back(std::make_pair(8,0.0));
	m_CalibrationRunList.push_back(std::make_pair(8,-m_VehicleInfo.max_wheel_angle/4.0));
	m_CalibrationRunList.push_back(std::make_pair(8,m_VehicleInfo.max_wheel_angle/3.0));
	m_CalibrationRunList.push_back(std::make_pair(8,0.0));
	m_CalibrationRunList.push_back(std::make_pair(8,-m_VehicleInfo.max_wheel_angle/3.0));

	m_CalibrationRunList.push_back(std::make_pair(0,0));

	m_CalibrationRunList.push_back(std::make_pair(9,0));
	m_CalibrationRunList.push_back(std::make_pair(9,m_VehicleInfo.max_wheel_angle/6.0));
	m_CalibrationRunList.push_back(std::make_pair(9,0.0));
	m_CalibrationRunList.push_back(std::make_pair(9,-m_VehicleInfo.max_wheel_angle/6.0));
	m_CalibrationRunList.push_back(std::make_pair(9,m_VehicleInfo.max_wheel_angle/4.0));
	m_CalibrationRunList.push_back(std::make_pair(9,0.0));
	m_CalibrationRunList.push_back(std::make_pair(9,-m_VehicleInfo.max_wheel_angle/4.0));
	m_CalibrationRunList.push_back(std::make_pair(9,m_VehicleInfo.max_wheel_angle/3.0));
	m_CalibrationRunList.push_back(std::make_pair(9,0.0));
	m_CalibrationRunList.push_back(std::make_pair(9,-m_VehicleInfo.max_wheel_angle/3.0));

	m_CalibrationRunList.push_back(std::make_pair(0,0));

	m_CalibrationRunList.push_back(std::make_pair(10,0));
	m_CalibrationRunList.push_back(std::make_pair(10,m_VehicleInfo.max_wheel_angle/6.0));
	m_CalibrationRunList.push_back(std::make_pair(10,0.0));
	m_CalibrationRunList.push_back(std::make_pair(10,-m_VehicleInfo.max_wheel_angle/6.0));
	m_CalibrationRunList.push_back(std::make_pair(10,m_VehicleInfo.max_wheel_angle/5.0));
	m_CalibrationRunList.push_back(std::make_pair(10,0.0));
	m_CalibrationRunList.push_back(std::make_pair(10,-m_VehicleInfo.max_wheel_angle/5.0));
	m_CalibrationRunList.push_back(std::make_pair(10,m_VehicleInfo.max_wheel_angle/4.0));
	m_CalibrationRunList.push_back(std::make_pair(10,0.0));
	m_CalibrationRunList.push_back(std::make_pair(10,-m_VehicleInfo.max_wheel_angle/4.0));

	m_CalibrationRunList.push_back(std::make_pair(0,0));

	m_CalibrationRunList.push_back(std::make_pair(11,0));
	m_CalibrationRunList.push_back(std::make_pair(11,m_VehicleInfo.max_wheel_angle/8.0));
	m_CalibrationRunList.push_back(std::make_pair(11,0.0));
	m_CalibrationRunList.push_back(std::make_pair(11,-m_VehicleInfo.max_wheel_angle/8.0));
	m_CalibrationRunList.push_back(std::make_pair(11,m_VehicleInfo.max_wheel_angle/6.0));
	m_CalibrationRunList.push_back(std::make_pair(11,0.0));
	m_CalibrationRunList.push_back(std::make_pair(11,-m_VehicleInfo.max_wheel_angle/6.0));
	m_CalibrationRunList.push_back(std::make_pair(11,m_VehicleInfo.max_wheel_angle/5.0));
	m_CalibrationRunList.push_back(std::make_pair(11,0.0));
	m_CalibrationRunList.push_back(std::make_pair(11,-m_VehicleInfo.max_wheel_angle/5.0));

	m_CalibrationRunList.push_back(std::make_pair(0,0));

	m_CalibrationRunList.push_back(std::make_pair(13,0));
	m_CalibrationRunList.push_back(std::make_pair(13,m_VehicleInfo.max_wheel_angle/10.0));
	m_CalibrationRunList.push_back(std::make_pair(13,0.0));
	m_CalibrationRunList.push_back(std::make_pair(13,-m_VehicleInfo.max_wheel_angle/10.0));
	m_CalibrationRunList.push_back(std::make_pair(13,m_VehicleInfo.max_wheel_angle/9.0));
	m_CalibrationRunList.push_back(std::make_pair(13,0.0));
	m_CalibrationRunList.push_back(std::make_pair(13,-m_VehicleInfo.max_wheel_angle/9.0));
	m_CalibrationRunList.push_back(std::make_pair(13,m_VehicleInfo.max_wheel_angle/8.0));
	m_CalibrationRunList.push_back(std::make_pair(13,0.0));
	m_CalibrationRunList.push_back(std::make_pair(13,-m_VehicleInfo.max_wheel_angle/8.0));

	m_CalibrationRunList.push_back(std::make_pair(0,0));

	m_CalibrationRunList.push_back(std::make_pair(15,0));
	m_CalibrationRunList.push_back(std::make_pair(15,m_VehicleInfo.max_wheel_angle/15.0));
	m_CalibrationRunList.push_back(std::make_pair(15,0.0));
	m_CalibrationRunList.push_back(std::make_pair(15,-m_VehicleInfo.max_wheel_angle/15.0));
	m_CalibrationRunList.push_back(std::make_pair(15,m_VehicleInfo.max_wheel_angle/12.0));
	m_CalibrationRunList.push_back(std::make_pair(15,0.0));
	m_CalibrationRunList.push_back(std::make_pair(15,-m_VehicleInfo.max_wheel_angle/12.0));
	m_CalibrationRunList.push_back(std::make_pair(15,m_VehicleInfo.max_wheel_angle/10.0));
	m_CalibrationRunList.push_back(std::make_pair(15,0.0));
	m_CalibrationRunList.push_back(std::make_pair(15,-m_VehicleInfo.max_wheel_angle/10.0));
	m_CalibrationRunList.push_back(std::make_pair(0,0));
}

} /* namespace SimulationNS */
