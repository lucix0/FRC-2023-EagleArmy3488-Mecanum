package frc.robot.commands.auto;

import static frc.robot.Constants.*;

import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.MecanumControllerCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.Trajectories;
import frc.robot.subsystems.DriveTrain;

public class TestOneAuto {
    private DriveTrain driveTrain;

    private PathPlannerTrajectory path;
    private TrajectoryConfig config;

    public TestOneAuto(Trajectories paths) {
        paths.getTrajectory("PathONE");
        config = new TrajectoryConfig(
            DriveConstants.kVelocityMax, 
            DriveConstants.kAccelerationMax
        );
    }

    public Command getCommand() {
        return new MecanumControllerCommand(
            path,
            driveTrain::getPose,
            driveTrain.getFeedForward(),
            driveTrain.getKinematics(), 
            
            // Position controllers
            new PIDController(0, 0, 0), 
            new PIDController(0, 0, 0), 
            new ProfiledPIDController(0, 0, 0, 
                DriveConstants.kThetaConstraints),

            DriveConstants.kAutoConstraints.maxVelocity,

            // Velocity PID
            driveTrain.getFLPID(),
            driveTrain.getBLPID(),
            driveTrain.getFRPID(),
            driveTrain.getBRPID(),
            driveTrain::getWheelSpeeds,
            driveTrain::setMotorVolts,
            driveTrain
        );
    }
}
