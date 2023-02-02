// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static class Controller {
        public static final int kDriverPort = 0;
        public static final int kOperatorPort = 1;
        public static final double kDeadzone = 0.15;
    }

    public static class Drive {
        // CAN ids.
        public static final int kFLMotorID = 3;
        public static final int kBLMotorID = 1;
        public static final int kFRMotorID = 4;
        public static final int kBRMotorID = 2;

        // Direction constants.
        public static final boolean kLeftInverted = false;
        public static final boolean kRightInverted = true;

        // Ramp val
        public static final double kRampInSec = 0.1875;

        // Physical Robot Properties (in inches)
        public static final double kWheelRadius = 3;
        public static final double kGearRatio = 10.71;
        public static final double kEncoderResolution = 2048;

        // Characterization Constants
        public static final double kP = 2.8287;
        public static final double kI = 0.0;
        public static final double kD = 0.0;
        public static final double kS = 0.054208;
        public static final double kV = 2.474;
        public static final double kA = 0.21718;

        public static final double kThetaP = 2.12312;

        public static final double kPosP = 85.171;
        public static final double kPosD = 5.3889;

        public static final Constraints kAutoConstraints = new Constraints(
            0.9144, // Max velocity     // 6ft/s
            0.6096  // Max acceleration // 4ft/s^2
        );

        public static final Constraints kThetaConstraints = new Constraints(
            Math.PI, 
            Math.PI 
        );
    }

    public static class Path {
        public static final String[] names = { "PathONE", "PathTWO", "Straight" };
    }

    public static class Extend {
        public static final double kMotor = 4;
        public static final double kP = 0;
        public static final double kI = 0;
        public static final double kD = 0;
    }
}
