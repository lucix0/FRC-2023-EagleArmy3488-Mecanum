package frc.robot.commands.auto;

import frc.robot.Robot;
import frc.robot.Constants.*;
import frc.robot.Trajectories;
import frc.robot.subsystems.DriveSubsystem;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.MecanumControllerCommand;

import com.pathplanner.lib.PathPlannerTrajectory;

public class TestOneAuto {
    private final DriveSubsystem m_DriveSubsystem;
    private PathPlannerTrajectory path;

    public TestOneAuto(Trajectories paths) {
        this.m_DriveSubsystem = Robot.m_DriveSubsystem;
        path = paths.getTrajectory("Straight");
    }

    public Command getCommand() {
        return new MecanumControllerCommand(
            path,
            m_DriveSubsystem::getPose,
            m_DriveSubsystem.getFeedForward(),
            m_DriveSubsystem.getKinematics(), 

            // Position controllers
            new PIDController(Drive.kPosP, 0, Drive.kPosD), 
            new PIDController(Drive.kPosP, 0, Drive.kPosD), 
            new ProfiledPIDController(Drive.kThetaP, 0, 0, 
                Drive.kThetaConstraints),

            Drive.kAutoConstraints.maxVelocity,

            // Velocity PID
            m_DriveSubsystem.getFLPID(),
            m_DriveSubsystem.getBLPID(),
            m_DriveSubsystem.getFRPID(),
            m_DriveSubsystem.getBRPID(),
            m_DriveSubsystem::getWheelSpeeds,
            m_DriveSubsystem::setMotorVolts,
            m_DriveSubsystem
        );
    }
}
