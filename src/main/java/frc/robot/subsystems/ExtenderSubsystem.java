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

public class ExtenderSubsystem extends PositionalSubsystem {
    private WPI_TalonFX extenderMotor1, extenderMotor2;
    private double currentPosition = Extender.kRetractedPosition;

    public ExtenderSubsystem() {
        extenderMotor1 = new WPI_TalonFX(RobotMap.kExtenderMotorID1);
        extenderMotor1.configFactoryDefault();
        extenderMotor1.setSelectedSensorPosition(0);
        extenderMotor1.setNeutralMode(NeutralMode.Brake);
        extenderMotor1.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor,
            Extender.kSlotIdx, Extender.kTimeoutMs);
        extenderMotor1.config_kF(Extender.kSlotIdx, Extender.kF);
        extenderMotor1.config_kP(Extender.kSlotIdx, Extender.kP);
        extenderMotor1.config_kI(Extender.kSlotIdx, Extender.kI);
        extenderMotor1.config_kD(Extender.kSlotIdx, Extender.kD);
        extenderMotor1.configMotionAcceleration(Extender.kMaxAcceleration, Extender.kTimeoutMs);
        extenderMotor1.configMotionCruiseVelocity(Extender.kMaxVelocity, Extender.kTimeoutMs);

        extenderMotor2 = new WPI_TalonFX(RobotMap.kExtenderMotorID2);
        extenderMotor2.configFactoryDefault();
        extenderMotor2.setSelectedSensorPosition(0);
        extenderMotor2.setNeutralMode(NeutralMode.Brake);
        extenderMotor2.setInverted(true);
        extenderMotor2.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor,
            Extender.kSlotIdx, Extender.kTimeoutMs);
        extenderMotor2.config_kF(Extender.kSlotIdx, Extender.kF);
        extenderMotor2.config_kP(Extender.kSlotIdx, Extender.kP);
        extenderMotor2.config_kI(Extender.kSlotIdx, Extender.kI);
        extenderMotor2.config_kD(Extender.kSlotIdx, Extender.kD);
        extenderMotor2.configMotionAcceleration(Extender.kMaxAcceleration, Extender.kTimeoutMs);
        extenderMotor2.configMotionCruiseVelocity(Extender.kMaxVelocity, Extender.kTimeoutMs);
        extenderMotor2.follow(extenderMotor1);
    }

    @Override
    public void run() {
        extenderMotor1.set(ControlMode.MotionMagic, currentPosition);
    }

    @Override
    public void changePosition(double position) {
        currentPosition = position;
    }
}
