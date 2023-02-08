package frc.robot.commands.auto;

import frc.lib.auto.Trajectories;
import frc.robot.Constants.*;
import frc.robot.subsystems.DriveSubsystem;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.MecanumControllerCommand;

import com.pathplanner.lib.PathPlannerTrajectory;


public class AutoTest1 {
    private DriveSubsystem m_DriveSubsystem;

    private PathPlannerTrajectory path;

    public AutoTest1(Trajectories paths, DriveSubsystem driveSubsystem) {  
        path = paths.getTrajectory("NotStraight");
        m_DriveSubsystem = driveSubsystem;
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
            m_DriveSubsystem.getDrivePID(0),
            m_DriveSubsystem.getDrivePID(2),
            m_DriveSubsystem.getDrivePID(1),
            m_DriveSubsystem.getDrivePID(3),
            m_DriveSubsystem::getWheelSpeeds,
            m_DriveSubsystem::setMotorVolts,
            m_DriveSubsystem
        );
    }
}
