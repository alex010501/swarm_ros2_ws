#pragma once

#include "RobotAdditions.h"

enum DriveTarget
{
	NoneTarget,
	PosTarget,
	VelTarget,
	ForceTarget
};

class amplifier
{
private:
	MathAdditions::Integrator integrator;
	double prevOutput;
	double Ky;
	double Ty;
public:
	void init(double p_Ky, double p_Ty, double p_dt)
	{
		this->Ky = p_Ky;
		this->Ty = p_Ty;
		this->integrator.init(p_dt, 0);
		this->prevOutput = 0;
	}

	double calculate(double p_input)
	{
		double output = this->integrator.calculate(Ky * (p_input - this->prevOutput) / Ty);
		this->prevOutput = output;
		return output;
	}
};

class driveCircuit
{
private:
	double L;
	double R;
	double prevCurrent;
	MathAdditions::Integrator integrator;

public:
	void init(double p_L, double p_R, double p_dt)
	{
		this->L = p_L;
		this->R = p_R;
		this->prevCurrent = 0;
		this->integrator.init(p_dt, 0);
	}

	double calculate(double p_U)
	{
		double U = p_U - this->prevCurrent * this->R;
		this->prevCurrent = this->integrator.calculate(U / this->L);
		return this->prevCurrent;
	}
};

class RobotJoint //Class of joints and motors of Robot 
{
private:
	// Joint coordinates
	double m_jointPos;
	double m_jointVel;
	double m_jointAcc;

	// Target coordinates
	DriveTarget m_jointTargetType;
	double m_jointTargetPos;
	double m_jointTargetVel;
	double m_jointTargetForce;

	// Joint parameters
	MoveType m_jointType;
	Bounds m_jointBounds;

	// Motor parameters
	double m_gearRatio;
	double m_motorPower;
	double m_motorSpeed;
	double m_motorSpeed_max;
	double m_motorTorque;
	double m_motorTorque_max;	
public:
	// Class constructor
	RobotJoint() = default;
	RobotJoint(MoveType p_j, double p_maxPos = PI, double p_minPos = -PI);
	// Dynamic mode switch
	void enableDynamicMode(double p_gearRatio, double p_motorPower, double p_motorSpeed, double p_motorSpeed_max, double p_motorTorque, double p_motorTorque_max);

	// Getters of joint parameters
	double getJointPos() const;
	double getJointVel() const;
	double getJointAcc() const;
	double getJointTarget() const;
	DriveTarget getJointTargetType() const;
	MoveType getJointType() const;

	// Setter of joint target
	void setJointTarget(DriveTarget _targetType, double _targetValue);
	// Setter of joint bounds
	void setJointBounds(double p_minPos, double p_maxPos);
};