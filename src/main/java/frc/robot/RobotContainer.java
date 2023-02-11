// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.lib.auto.Trajectories;
import frc.robot.commands.DriveCmd;
import frc.robot.commands.RunCmd;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ExtenderSubsystem;
import frc.robot.subsystems.FourBarSubsystem;
import frc.robot.subsystems.GrabberSubsystem;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;

public class RobotContainer {
    private final DriveSubsystem m_DriveSubsystem;
    private final ExtenderSubsystem m_ExtenderSubsystem;
    private final FourBarSubsystem m_FourBarSubsystem;
    private final GrabberSubsystem m_GrabberSubsystem;

    private final Controls controls;
    private final UI ui;

    private final Trajectories paths;

    public RobotContainer() {
        m_DriveSubsystem = new DriveSubsystem();
        m_ExtenderSubsystem = new ExtenderSubsystem();
        m_FourBarSubsystem = new FourBarSubsystem();
        m_GrabberSubsystem = new GrabberSubsystem();

        controls = new Controls(m_DriveSubsystem, m_ExtenderSubsystem, 
                                m_FourBarSubsystem, m_GrabberSubsystem);

        configureBindings();

        // Autonomous routines.
        paths = new Trajectories();

        ui = new UI(m_DriveSubsystem, m_ExtenderSubsystem, 
                    m_FourBarSubsystem, m_GrabberSubsystem, paths);

        m_DriveSubsystem.setDefaultCommand(new DriveCmd(m_DriveSubsystem, () -> controls.getDriverController().getLeftY(),
                () -> controls.getDriverController().getLeftX(), () -> controls.getDriverController().getRightX()));
        m_ExtenderSubsystem.setDefaultCommand(new RunCmd(m_ExtenderSubsystem));
        m_FourBarSubsystem.setDefaultCommand(new RunCmd(m_FourBarSubsystem));
    }

    private void configureBindings() {  }

    public Command getAutonomousCommand() {
        m_DriveSubsystem.resetOdometry(new Pose2d());
        return ui.getChosenAuto();
    }
}
