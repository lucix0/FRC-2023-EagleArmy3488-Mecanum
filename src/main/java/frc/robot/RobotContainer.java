// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.Controller;
import frc.robot.commands.FourBarActCmd;
import frc.robot.commands.GrabCmd;
import frc.robot.commands.MecanumDriveCmd;
import frc.robot.commands.SetOrientationCmd;
import frc.robot.commands.auto.TestOneAuto;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.FourBar;
import frc.robot.subsystems.Grabber;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class RobotContainer {
    // Subsystems.
    private final DriveTrain driveTrain;
    private final Grabber grabber;
    private final FourBar fourBar;

    // Autonomous routines.
    private final Trajectories paths;
    private final SendableChooser<Command> routineChooser;
    private final TestOneAuto testRoutine1;

    private final XboxController driveController;
    private final XboxController operatorController;

    public RobotContainer() {
        driveTrain = new DriveTrain();
        grabber = new Grabber();
        fourBar = new FourBar();

        driveController = new XboxController(Controller.kDriverPort);
        operatorController = new XboxController(Controller.kOperatorPort);
        configureBindings();

        // Autonomous routines.
        paths = new Trajectories();
        testRoutine1 = new TestOneAuto(paths, driveTrain);

        // Sets up dashboard for choosing different auto routines on the fly.
        routineChooser = new SendableChooser<>();
        routineChooser.setDefaultOption("None", null);
        routineChooser.addOption("Test Routine 1", testRoutine1.getCommand());
        SmartDashboard.putData("Auto Routines", routineChooser);

        driveTrain.setDefaultCommand(new MecanumDriveCmd(driveTrain, () -> driveController.getLeftY(),
                () -> driveController.getLeftX(), () -> driveController.getRightX()));
    }

    private void configureBindings() {
        Trigger drLeftTrigger = new Trigger(() -> driveController.getLeftTriggerAxis() >= 0.1);
        Trigger drRightTrigger = new Trigger(() -> driveController.getRightTriggerAxis() >= 0.1);

        Trigger opLeftTrigger = new Trigger(() -> operatorController.getLeftTriggerAxis() >= 0.1);
        Trigger opRightTrigger = new Trigger(() -> operatorController.getRightTriggerAxis() >= 0.1);

        // Driver.
        new JoystickButton(driveController, XboxController.Button.kLeftStick.value)
            .onTrue(new SetOrientationCmd(driveTrain));
        
        // Operator.
        opRightTrigger.whileTrue(new GrabCmd(grabber));
        new JoystickButton(operatorController, XboxController.Button.kRightBumper.value)
            .onTrue(new FourBarActCmd(fourBar));
    }

    public Command getAutonomousCommand() {
        driveTrain.resetOdometry(new Pose2d());
        return routineChooser.getSelected();
    }
}
