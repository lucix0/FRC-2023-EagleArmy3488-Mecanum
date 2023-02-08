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
import frc.robot.RobotMap;
import frc.robot.Constants.*;

public class ExtenderSubsystem extends SubsystemBase {
    private final WPI_TalonFX extenderMotorOne, extenderMotorTwo;
    private double speed;

    public ExtenderSubsystem() {
        speed = Extender.kRaiseSpeed;
        extenderMotorOne = new WPI_TalonFX(RobotMap.kExtenderMotorID1);
        extenderMotorOne.configFactoryDefault();

        extenderMotorTwo = new WPI_TalonFX(RobotMap.kExtenderMotorID2);
        extenderMotorTwo.configFactoryDefault();
        extenderMotorTwo.follow(extenderMotorOne);

        extenderMotorOne.setNeutralMode(NeutralMode.Brake);
        extenderMotorTwo.setNeutralMode(NeutralMode.Brake);

        // PID
        extenderMotorOne.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor,
                Extender.kSlotIdx, Extender.kTimeoutMs);
        extenderMotorOne.config_kF(Extender.kSlotIdx, Extender.kF);
        extenderMotorOne.config_kP(Extender.kSlotIdx, Extender.kP);
        extenderMotorOne.config_kI(Extender.kSlotIdx, Extender.kI);
        extenderMotorOne.config_kD(Extender.kSlotIdx, Extender.kD);
        extenderMotorTwo.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor,
                Extender.kSlotIdx, Extender.kTimeoutMs);
        extenderMotorTwo.config_kF(Extender.kSlotIdx, Extender.kF);
        extenderMotorTwo.config_kP(Extender.kSlotIdx, Extender.kP);
        extenderMotorTwo.config_kI(Extender.kSlotIdx, Extender.kI);
        extenderMotorTwo.config_kD(Extender.kSlotIdx, Extender.kD);
    }

    public void run() {
        if (speed == Extender.kRaiseSpeed) {
            extenderMotorOne.set(ControlMode.Position, Extender.kRaiseLimit);
            extenderMotorTwo.set(ControlMode.Position, Extender.kRaiseLimit);
        }
        if (speed == Extender.kDropSpeed) {
            extenderMotorOne.set(ControlMode.Position, Extender.kDropLimit);
            extenderMotorTwo.set(ControlMode.Position, Extender.kDropLimit);
        }
    }

    public void setToRaise() {
        speed = Extender.kRaiseSpeed;
    }

    public void setToLower() {
        speed = Extender.kDropSpeed;

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
