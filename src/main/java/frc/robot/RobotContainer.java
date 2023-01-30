// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.ControllerConstants;
import frc.robot.commands.MecanumDriveCmd;
import frc.robot.commands.SetOrientationCmd;
import frc.robot.commands.auto.TestOneAuto;
import frc.robot.subsystems.DriveTrain;

import com.fasterxml.jackson.databind.node.POJONode;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
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

    controller = new XboxController(ControllerConstants.kPort);
    configureBindings();

    // Autonomous routines.
    paths = new Trajectories();
    testRoutine1 = new TestOneAuto(paths, driveTrain);

    // Sets up dashboard for choosing different auto routines on the fly.
    routineChooser = new SendableChooser<>();
    routineChooser.setDefaultOption("None", null);
    routineChooser.addOption("Test Routine 1", testRoutine1.getCommand());
    SmartDashboard.putData(routineChooser);
    
    driveTrain.setDefaultCommand(new MecanumDriveCmd(driveTrain, () -> controller.getLeftY(), () -> controller.getLeftX(), () -> controller.getRightX()));
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    new JoystickButton(controller, XboxController.Button.kLeftStick.value).
      whileTrue(new SetOrientationCmd(driveTrain));
  }

  public Command getAutonomousCommand() {
    driveTrain.resetOdometry(new Pose2d());
    return routineChooser.getSelected();
  }
}
