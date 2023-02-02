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

public class FourBar extends SubsystemBase {
    private WPI_TalonFX fourBarMotorOne, fourBarMotorTwo;
    private double speed;

    public FourBar() {
        speed = FB.kUpperSpeed;
        fourBarMotorOne = new WPI_TalonFX(FB.kFourBarMotorOne);
        fourBarMotorOne.configFactoryDefault();

        fourBarMotorTwo = new WPI_TalonFX(FB.kFourBarMotorTwo);
        fourBarMotorTwo.configFactoryDefault();
        fourBarMotorTwo.follow(fourBarMotorOne);

        fourBarMotorOne.setNeutralMode(NeutralMode.Brake);
        fourBarMotorTwo.setNeutralMode(NeutralMode.Brake);

        // PID
        fourBarMotorOne.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, FB.kSlotIdx,
                FB.kTimeoutMs);
        fourBarMotorOne.config_kF(FB.kSlotIdx, FB.kF);
        fourBarMotorOne.config_kP(FB.kSlotIdx, FB.kP);
        fourBarMotorOne.config_kI(FB.kSlotIdx, FB.kI);
        fourBarMotorOne.config_kD(FB.kSlotIdx, FB.kD);
        fourBarMotorTwo.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, FB.kSlotIdx,
                FB.kTimeoutMs);
        fourBarMotorTwo.config_kF(FB.kSlotIdx, FB.kF);
        fourBarMotorTwo.config_kP(FB.kSlotIdx, FB.kP);
        fourBarMotorTwo.config_kI(FB.kSlotIdx, FB.kI);
        fourBarMotorTwo.config_kD(FB.kSlotIdx, FB.kD);
    }

    public void run() {
        if (speed == FB.kUpperSpeed) {
            fourBarMotorOne.set(ControlMode.Position, FB.kUpperLimit);
            fourBarMotorTwo.set(ControlMode.Position, FB.kUpperLimit);
            setToLower();
        } else if (speed == FB.kLowerSpeed) {
            fourBarMotorOne.set(ControlMode.Position, FB.kLowerLimit);
            fourBarMotorTwo.set(ControlMode.Position, FB.kLowerLimit);
            setToRaise();
        }
    }

    public void setToRaise() {
        speed = FB.kUpperSpeed;
    }

    public void setToLower() {
        speed = FB.kLowerSpeed;

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
