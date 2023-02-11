package frc.robot.commands.auto;

import frc.lib.auto.Trajectories;
import frc.robot.Constants.*;
import frc.robot.commands.SetFollowCmd;
import frc.robot.subsystems.DriveSubsystem;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.MecanumControllerCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import com.pathplanner.lib.PathPlannerTrajectory;


public class AutoTest1 {
    private DriveSubsystem m_DriveSubsystem;

    private PathPlannerTrajectory path;

    public AutoTest1(Trajectories paths, DriveSubsystem driveSubsystem) {  
        path = paths.getTrajectory("NotStraight");
        m_DriveSubsystem = driveSubsystem;
        m_DriveSubsystem.setFollow();
    }

    public Command getCommand() {
        return new SequentialCommandGroup(
            new RamseteCommand(
                path, 
                m_DriveSubsystem::getPose, 
                new RamseteController(Drive.kRamseteB, Drive.kRamseteZeta), 
                m_DriveSubsystem.getFeedForward(), 
                m_DriveSubsystem.getTankKinematics(), 
                m_DriveSubsystem::getTankWheelSpeeds,
                m_DriveSubsystem.getLeftPID(),
                m_DriveSubsystem.getRightPID(),
                m_DriveSubsystem::setTankVolts,
                m_DriveSubsystem
            ),
            new SetFollowCmd(m_DriveSubsystem)
        );
    }
}
