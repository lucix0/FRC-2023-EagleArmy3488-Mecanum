// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.lib.auto.Trajectories;
import frc.robot.commands.DriveCmd;
import frc.robot.commands.auto.TestOneAuto;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ExtenderSubsystem;
import frc.robot.subsystems.FourBarSubsystem;
import frc.robot.subsystems.GrabberSubsystem;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class RobotContainer {
    private final DriveSubsystem m_DriveSubsystem;
    private final ExtenderSubsystem m_ExtenderSubsystem;
    private final FourBarSubsystem m_FourBarSubsystem;
    private final GrabberSubsystem m_GrabberSubsystem;

    private final Controls controls;

    private final Trajectories paths;
    private final SendableChooser<Command> routineChooser;
    private final TestOneAuto testRoutine1;

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
        testRoutine1 = new TestOneAuto(paths, m_DriveSubsystem);

        // Sets up dashboard for choosing different auto routines on the fly.
        routineChooser = new SendableChooser<>();
        routineChooser.setDefaultOption("None", null);
        routineChooser.addOption("Test Routine 1", testRoutine1.getCommand());
        SmartDashboard.putData("Auto Routines", routineChooser);

        m_DriveSubsystem.setDefaultCommand(new DriveCmd(m_DriveSubsystem, () -> controls.getDriverController().getLeftY(),
                () -> controls.getDriverController().getLeftX(), () -> controls.getDriverController().getRightX()));
    }

    private void configureBindings() {  }

    public Command getAutonomousCommand() {
        m_DriveSubsystem.resetOdometry(new Pose2d());
        return routineChooser.getSelected();
    }
}
