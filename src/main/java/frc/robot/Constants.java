// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
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
        public static final double kDeadband = 0.15;
    }

    public static class Drive {
        // Direction constants.
        public static final boolean kLeftInverted = false;
        public static final boolean kRightInverted = true;

        // Ramp val
        public static final double kRampInSec = 0.1875;

        // Physical Robot Properties (in inches)
        public static final double kWheelRadius = 4;
        public static final double kGearRatio = 12.75;
        public static final double kEncoderResolution = 2048;
        public static final double kCenterToWheel = 0.305;
        public static final MecanumDriveKinematics kKinematics = new MecanumDriveKinematics(
            new Translation2d( Drive.kCenterToWheel,  Drive.kCenterToWheel), 
            new Translation2d( Drive.kCenterToWheel, -Drive.kCenterToWheel), 
            new Translation2d(-Drive.kCenterToWheel,  Drive.kCenterToWheel), 
            new Translation2d(-Drive.kCenterToWheel, -Drive.kCenterToWheel)
        );

        // Characterization Constants
        public static final double kP = 2.9237;
        public static final double kI = 0.0;
        public static final double kD = 0.0;
        public static final double kS = 0.058218;
        public static final double kV = 2.191;
        public static final double kA = 0.38399;

        public static final double kThetaP = 2.12312;

        public static final double kPosP = 71.772;
        public static final double kPosD = 6.6952;

        public static final Constraints kAutoConstraints = new Constraints(
            2.5, // Max velocity     // 6ft/s
            2  // Max acceleration // 4ft/s^2
        );
    }

    public static class Balance {
        public static final double kGoalDegrees = 0.0;
        public static final double kP = 0.015;
        public static final double kBackwardsBoostMultiplier = 1.0;
        public static final double kAngleThreshold = 3.0;
    }

    public static class Turn {
        public static final double kP = 0.007;
        public static final double kAngleThreshold = 3.0;
    }

    public static class Path {
        public static final String[] names = { "LeaveCommunity" };
    }

    public static class FourBar {
        public static final int kSlotIdx = 0;
        
        public static final double kF = 0;
        public static final double kP = 0.1;
        public static final double kI = 0;
        public static final double kD = 0;
    
        public static final int kTimeoutMs = 30;
    
        public static final double kRetractedPosition = 0;
        public static final double kExtendedPosition = 43000;
        
        public static final double kMaxVelocity = 20000;
        public static final double kMaxAcceleration = 1000;
    }

    public static class Extender {
        public static final int kSlotIdx = 0;
    
        public static final double kF = 0;
        public static final double kP = 0.1;
        public static final double kI = 0;
        public static final double kD = 0;

        public static final int kTimeoutMs = 30;

        public static final double kRetractedPosition = 0;
        public static final double kExtendedGrabPosition = 28000;
        public static final double kExtendedPlacePosition = 34500;

        public static final double kMaxVelocity = 20000;
        public static final double kMaxAcceleration = 2000;
    }

    public static class Grabber {
        public static final double kGrabSpeed = -0.6;
        public static final double kDropSpeed = 0.2;
    }
}
