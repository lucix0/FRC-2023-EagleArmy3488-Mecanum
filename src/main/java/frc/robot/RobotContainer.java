// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.Controller;
import frc.robot.commands.MecanumDriveCmd;
import frc.robot.commands.SetOrientationCmd;
import frc.robot.commands.auto.TestOneAuto;
import frc.robot.subsystems.DriveTrain;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class RobotContainer {
    // Subsystems.
    private final DriveTrain driveTrain;

    // Autonomous routines.
    private final Trajectories paths;
    private final SendableChooser<Command> routineChooser;
    private final TestOneAuto testRoutine1;

    private final XboxController controller;

    public RobotContainer() {
        driveTrain = new DriveTrain();

        controller = new XboxController(Controller.kPort);
        configureBindings();

        // Autonomous routines.
        paths = new Trajectories();
        testRoutine1 = new TestOneAuto(paths, driveTrain);

        // Sets up dashboard for choosing different auto routines on the fly.
        routineChooser = new SendableChooser<>();
        routineChooser.setDefaultOption("None", null);
        routineChooser.addOption("Test Routine 1", testRoutine1.getCommand());
        SmartDashboard.putData(routineChooser);

        driveTrain.setDefaultCommand(new MecanumDriveCmd(driveTrain, () -> controller.getLeftY(),
                () -> controller.getLeftX(), () -> controller.getRightX()));
    }

    private void configureBindings() {
        new JoystickButton(controller, XboxController.Button.kLeftStick.value)
            .onTrue(new SetOrientationCmd(driveTrain));
    }

    public Command getAutonomousCommand() {
        driveTrain.resetOdometry(new Pose2d());
        return routineChooser.getSelected();
    }
}
