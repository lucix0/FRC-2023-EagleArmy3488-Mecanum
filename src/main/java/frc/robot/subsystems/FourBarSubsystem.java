// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.RobotMap;
import frc.robot.Constants.*;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

public class FourBarSubsystem extends PositionalSubsystem {
    private WPI_TalonFX fourBarMotor1, fourBarMotor2;
    private double currentPosition = FourBar.kRetractedPosition;

    public FourBarSubsystem() {
        fourBarMotor1 = new WPI_TalonFX(RobotMap.kFourBarMotorID1);
        fourBarMotor1.configFactoryDefault();
        fourBarMotor1.setSelectedSensorPosition(0);
        fourBarMotor1.setNeutralMode(NeutralMode.Brake);
        fourBarMotor1.setInverted(true);
        fourBarMotor1.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, FourBar.kSlotIdx,
            FourBar.kTimeoutMs);
        fourBarMotor1.config_kF(FourBar.kSlotIdx, FourBar.kF);
        fourBarMotor1.config_kP(FourBar.kSlotIdx, FourBar.kP);
        fourBarMotor1.config_kI(FourBar.kSlotIdx, FourBar.kI);
        fourBarMotor1.config_kD(FourBar.kSlotIdx, FourBar.kD);
        fourBarMotor1.configMotionAcceleration(FourBar.kMaxAcceleration, FourBar.kTimeoutMs);
        fourBarMotor1.configMotionCruiseVelocity(FourBar.kMaxVelocity, FourBar.kTimeoutMs);

        fourBarMotor2 = new WPI_TalonFX(RobotMap.kFourBarMotorID2);
        fourBarMotor2.configFactoryDefault();
        fourBarMotor2.setSelectedSensorPosition(0);
        fourBarMotor2.setNeutralMode(NeutralMode.Brake);
        fourBarMotor2.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, FourBar.kSlotIdx,
            FourBar.kTimeoutMs);
        
        fourBarMotor2.config_kF(FourBar.kSlotIdx, FourBar.kF);
        fourBarMotor2.config_kP(FourBar.kSlotIdx, FourBar.kP);
        fourBarMotor2.config_kI(FourBar.kSlotIdx, FourBar.kI);
        fourBarMotor2.config_kD(FourBar.kSlotIdx, FourBar.kD);
        fourBarMotor2.configMotionAcceleration(FourBar.kMaxAcceleration, FourBar.kTimeoutMs);
        fourBarMotor2.configMotionCruiseVelocity(FourBar.kMaxVelocity, FourBar.kTimeoutMs);
        fourBarMotor2.follow(fourBarMotor1);
    }

    @Override
    public void run() {
        fourBarMotor1.set(ControlMode.MotionMagic, currentPosition);
    }

    @Override
    public void changePosition(double position) {
        currentPosition = position;
    }
}
