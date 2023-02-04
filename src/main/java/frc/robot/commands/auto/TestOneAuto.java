package frc.robot.commands.auto;

import static frc.robot.Constants.*;

import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.MecanumControllerCommand;
import frc.robot.Trajectories;
import frc.robot.subsystems.DriveSubsystem;

public class TestOneAuto {
    private DriveSubsystem driveTrain;

    private PathPlannerTrajectory path;

    public TestOneAuto(Trajectories paths, DriveSubsystem driveTrain) {
        this.driveTrain = driveTrain;
        path = paths.getTrajectory("Straight");
    }

    public Command getCommand() {
        return new MecanumControllerCommand(
            path.transformBy(driveTrain.getPose().minus(path.getInitialPose())),
            driveTrain::getPose,
            driveTrain.getFeedForward(),
            driveTrain.getKinematics(), 

            // Position controllers
            new PIDController(Drive.kPosP, 0, Drive.kPosD), 
            new PIDController(Drive.kPosP, 0, Drive.kPosD), 
            new ProfiledPIDController(Drive.kThetaP, 0, 0, 
                Drive.kThetaConstraints),

            Drive.kAutoConstraints.maxVelocity,

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
