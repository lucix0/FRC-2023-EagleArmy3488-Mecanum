package frc.lib;

import frc.robot.Constants.*;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.util.Units;

public class MotorUtil {
    public static double getMotorDistance(WPI_TalonFX motor) {
        return ((double) getEncoderPosition(motor)) / Drive.kEncoderResolution / Drive.kGearRatio * 
            2 * Math.PI * Units.inchesToMeters(Drive.kWheelRadius);
    }

    public static double getEncoderPosition(WPI_TalonFX motor) {
        return motor.getSelectedSensorPosition();
    }

    public static double getMotorVelocity(WPI_TalonFX motor) {
        return motor.getSelectedSensorVelocity();
    }

    public static void resetEncoder(WPI_TalonFX motor) {
        motor.setSelectedSensorPosition(0);
    }

    public static void resetEncoders(WPI_TalonFX[] motors) {
        for (WPI_TalonFX motor : motors) {
            motor.setSelectedSensorPosition(0);
        }
    }

    public static void setFactory(WPI_TalonFX motor) {
        motor.configFactoryDefault();
    }
}
