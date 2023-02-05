// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants.*;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class FourBarSubsystem extends SubsystemBase {
    private WPI_TalonFX fourBarMotorOne, fourBarMotorTwo;
    private double speed;

    public FourBarSubsystem() {
        speed = FourBar.kUpperSpeed;
        fourBarMotorOne = new WPI_TalonFX(FourBar.kMotorOneID);
        fourBarMotorOne.configFactoryDefault();

        fourBarMotorTwo = new WPI_TalonFX(FourBar.kMotorTwoID);
        fourBarMotorTwo.configFactoryDefault();
        fourBarMotorTwo.follow(fourBarMotorOne);

        fourBarMotorOne.setNeutralMode(NeutralMode.Brake);
        fourBarMotorTwo.setNeutralMode(NeutralMode.Brake);

        // PID
        fourBarMotorOne.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, FourBar.kSlotIdx,
                FourBar.kTimeoutMs);
        fourBarMotorOne.config_kF(FourBar.kSlotIdx, FourBar.kF);
        fourBarMotorOne.config_kP(FourBar.kSlotIdx, FourBar.kP);
        fourBarMotorOne.config_kI(FourBar.kSlotIdx, FourBar.kI);
        fourBarMotorOne.config_kD(FourBar.kSlotIdx, FourBar.kD);
        fourBarMotorTwo.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, FourBar.kSlotIdx,
                FourBar.kTimeoutMs);
        fourBarMotorTwo.config_kF(FourBar.kSlotIdx, FourBar.kF);
        fourBarMotorTwo.config_kP(FourBar.kSlotIdx, FourBar.kP);
        fourBarMotorTwo.config_kI(FourBar.kSlotIdx, FourBar.kI);
        fourBarMotorTwo.config_kD(FourBar.kSlotIdx, FourBar.kD);
    }

    public void run() {
        if (speed == FourBar.kUpperSpeed) {
            fourBarMotorOne.set(ControlMode.Position, FourBar.kUpperLimit);
            fourBarMotorTwo.set(ControlMode.Position, FourBar.kUpperLimit);
            setToLower();
        } else if (speed == FourBar.kLowerSpeed) {
            fourBarMotorOne.set(ControlMode.Position, FourBar.kLowerLimit);
            fourBarMotorTwo.set(ControlMode.Position, FourBar.kLowerLimit);
            setToRaise();
        }
    }

    public void setToRaise() {
        speed = FourBar.kUpperSpeed;
    }

    public void setToLower() {
        speed = FourBar.kLowerSpeed;

    }

    public void setSpeed(double speed) {
        this.speed = speed;
    }

    public void stop() {
        fourBarMotorTwo.set(0);
        fourBarMotorOne.set(0);
    }

    public double getSpeed() {
        return speed;
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        SmartDashboard.putNumber("Upwards Position", fourBarMotorOne.getSelectedSensorPosition());
        SmartDashboard.putNumber("Downwards Position", fourBarMotorTwo.getSelectedSensorPosition());
    }
}
