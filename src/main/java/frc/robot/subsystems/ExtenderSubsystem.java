// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.*;

public class ExtenderSubsystem extends SubsystemBase {
    private final WPI_TalonFX extenderMotorOne, extenderMotorTwo;
    private double speed;

    public ExtenderSubsystem() {
        speed = Extend.kRaiseSpeed;
        extenderMotorOne = new WPI_TalonFX(Extend.kExtenderMotorOne);
        extenderMotorOne.configFactoryDefault();

        extenderMotorTwo = new WPI_TalonFX(Extend.kExtenderMotorTwo);
        extenderMotorTwo.configFactoryDefault();
        extenderMotorTwo.follow(extenderMotorOne);

        extenderMotorOne.setNeutralMode(NeutralMode.Brake);
        extenderMotorTwo.setNeutralMode(NeutralMode.Brake);

        // PID
        extenderMotorOne.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor,
                Extend.kSlotIdx, Extend.kTimeoutMs);
        extenderMotorOne.config_kF(Extend.kSlotIdx, Extend.kF);
        extenderMotorOne.config_kP(Extend.kSlotIdx, Extend.kP);
        extenderMotorOne.config_kI(Extend.kSlotIdx, Extend.kI);
        extenderMotorOne.config_kD(Extend.kSlotIdx, Extend.kD);
        extenderMotorTwo.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor,
                Extend.kSlotIdx, Extend.kTimeoutMs);
        extenderMotorTwo.config_kF(Extend.kSlotIdx, Extend.kF);
        extenderMotorTwo.config_kP(Extend.kSlotIdx, Extend.kP);
        extenderMotorTwo.config_kI(Extend.kSlotIdx, Extend.kI);
        extenderMotorTwo.config_kD(Extend.kSlotIdx, Extend.kD);
    }

    public void run() {
        if (speed == Extend.kRaiseSpeed) {
            extenderMotorOne.set(ControlMode.Position, Extend.kRaiseLimit);
            extenderMotorTwo.set(ControlMode.Position, Extend.kRaiseLimit);
        }
        if (speed == Extend.kDropSpeed) {
            extenderMotorOne.set(ControlMode.Position, Extend.kDropLimit);
            extenderMotorTwo.set(ControlMode.Position, Extend.kDropLimit);
        }
    }

    public void setToRaise() {
        speed = Extend.kRaiseSpeed;
    }

    public void setToLower() {
        speed = Extend.kDropSpeed;

    }

    public void setSpeed(double speed) {
        this.speed = speed;
    }

    public void stop() {
        extenderMotorTwo.set(0);
        extenderMotorOne.set(0);
    }

    public double getSpeedValue() {
        return speed;
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        SmartDashboard.putNumber("Raised Position", extenderMotorOne.getSelectedSensorPosition());
        SmartDashboard.putNumber("Dropped Position", extenderMotorTwo.getSelectedSensorPosition());
    }
}
