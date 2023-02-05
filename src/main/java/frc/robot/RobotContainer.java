// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.Controller;
import frc.robot.commands.FourBarActCmd;
import frc.robot.commands.GrabCmd;
import frc.robot.commands.DriveCmd;
import frc.robot.commands.SetOrientationCmd;
import frc.robot.commands.auto.TestOneAuto;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.FourBarSubsystem;
import frc.robot.subsystems.GrabberSubsystem;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class RobotContainer {
    private final Config config;

    private final DriveSubsystem driveTrain;

    private final Trajectories paths;
    private final SendableChooser<Command> routineChooser;
    private final TestOneAuto testRoutine1;

    private final XboxController driveController;
    private final XboxController operatorController;

    public RobotContainer() {
        config = new Config("lonedrive");

        driveTrain = new DriveSubsystem();

        if (config.getDriveControllerStatus()) {
            driveController = new XboxController(Controller.kDriverPort);
        } else {
            driveController = null;
        }
        
        if (config.getOperatorControllerStatus()) {
            operatorController = new XboxController(Controller.kOperatorPort);
        } else {
            operatorController = null;
        }
        
        configureBindings();

        // Autonomous routines.
        paths = new Trajectories();
        testRoutine1 = new TestOneAuto(paths, driveTrain);

        // Sets up dashboard for choosing different auto routines on the fly.
        routineChooser = new SendableChooser<>();
        routineChooser.setDefaultOption("None", null);
        routineChooser.addOption("Test Routine 1", testRoutine1.getCommand());
        SmartDashboard.putData("Auto Routines", routineChooser);

        driveTrain.setDefaultCommand(new DriveCmd(driveTrain, () -> driveController.getLeftY(),
                () -> driveController.getLeftX(), () -> driveController.getRightX()));
    }

    private void configureBindings() {
        Trigger drLeftTrigger;
        Trigger drRightTrigger;
        if (config.getDriveControllerStatus()) {
            drLeftTrigger = new Trigger(() -> driveController.getLeftTriggerAxis() >= 0.1);
            drRightTrigger = new Trigger(() -> driveController.getRightTriggerAxis() >= 0.1);
        } else {
            drLeftTrigger = null;
            drRightTrigger = null;
        }

        Trigger opLeftTrigger;
        Trigger opRightTrigger;
        if (config.getOperatorControllerStatus()) {
            opLeftTrigger = new Trigger(() -> operatorController.getLeftTriggerAxis() >= 0.1);
            opRightTrigger = new Trigger(() -> operatorController.getRightTriggerAxis() >= 0.1);
        } else {
            opLeftTrigger = null;
            opRightTrigger = null;
        }

        if (config.getDriveControllerStatus()) {
            new JoystickButton(driveController, XboxController.Button.kLeftStick.value)
                .onTrue(new SetOrientationCmd(driveTrain));
        }
        
        if (config.getOperatorControllerStatus()) {

        }  
    }

    public Command getAutonomousCommand() {
        driveTrain.resetOdometry(new Pose2d());
        return routineChooser.getSelected();
    }
}
