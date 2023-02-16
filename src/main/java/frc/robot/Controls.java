package frc.robot;

import frc.robot.Constants.Grabber;
import frc.robot.commands.BrakeCmd;
import frc.robot.commands.ChangePositionCmd;
import frc.robot.commands.RunGrabberCmd;
import frc.robot.commands.SetGrabberSpeedCmd;
import frc.robot.commands.SetOrientationCmd;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ExtenderSubsystem;
import frc.robot.subsystems.FourBarSubsystem;
import frc.robot.subsystems.GrabberSubsystem;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/* Controller buttons are created and their actions are defined. */
public class Controls {
    // References to subsystems.
    private final DriveSubsystem m_DriveSubsystem;
    private final ExtenderSubsystem m_ExtenderSubsystem;
    private final FourBarSubsystem m_FourBarSubsystem;
    private final GrabberSubsystem m_GrabberSubsystem;

    private final XboxController driverController = new XboxController(RobotMap.kDriverControllerPort);
    private final XboxController operatorController = new XboxController(RobotMap.kDriverControllerPort);

    private final JoystickButton dA = new JoystickButton(driverController, 1);
    private final JoystickButton dB = new JoystickButton(driverController, 2);
    private final JoystickButton dX = new JoystickButton(driverController, 3);
    private final JoystickButton dY = new JoystickButton(driverController, 4);
    private final JoystickButton dLeftBumper = new JoystickButton(driverController, 5);
    private final JoystickButton dRightBumper = new JoystickButton(driverController, 6);
    private final JoystickButton dBack = new JoystickButton(driverController, 7);
    private final JoystickButton dStart = new JoystickButton(driverController, 8);
    private final JoystickButton dLStickDown = new JoystickButton(driverController, 9);
    private final JoystickButton dRStickDown = new JoystickButton(driverController, 10);
    private final Trigger dLeftTrigger = new Trigger(() -> driverController.getLeftTriggerAxis() >= 0.1);
    private final Trigger dRightTrigger = new Trigger(() -> driverController.getRightTriggerAxis() >= 0.1);
    private final POVButton dDPadUp = new POVButton(driverController, 0);
    private final POVButton dDPadRight = new POVButton(driverController, 90);
    private final POVButton dDPadDown = new POVButton(driverController, 180);
    private final POVButton dDPadLeft = new POVButton(driverController, 270);

    private final JoystickButton oA = new JoystickButton(operatorController, 1);
    private final JoystickButton oB = new JoystickButton(operatorController, 2);
    private final JoystickButton oX = new JoystickButton(operatorController, 3);
    private final JoystickButton oY = new JoystickButton(operatorController, 4);
    private final JoystickButton oLeftBumper = new JoystickButton(operatorController, 5);
    private final JoystickButton oRightBumper = new JoystickButton(operatorController, 6);
    private final JoystickButton oBack = new JoystickButton(operatorController, 7);
    private final JoystickButton oStart = new JoystickButton(operatorController, 8);
    private final JoystickButton oLStickDown = new JoystickButton(operatorController, 9);
    private final JoystickButton oRStickDown = new JoystickButton(operatorController, 10);
    private final Trigger oLeftTrigger = new Trigger(() -> operatorController.getLeftTriggerAxis() >= 0.1);
    private final Trigger oRightTrigger = new Trigger(() -> operatorController.getRightTriggerAxis() >= 0.1);
    private final POVButton oDPadUp = new POVButton(operatorController, 0);
    private final POVButton oDPadRight = new POVButton(operatorController, 90);
    private final POVButton oDPadDown = new POVButton(operatorController, 180);
    private final POVButton oDPadLeft = new POVButton(operatorController, 270);

    public Controls(DriveSubsystem driveSubsystem, ExtenderSubsystem extenderSubsystem, 
                    FourBarSubsystem fourBarSubsystem, GrabberSubsystem grabberSubsystem) {
        m_DriveSubsystem = driveSubsystem;
        m_ExtenderSubsystem = extenderSubsystem;
        m_FourBarSubsystem = fourBarSubsystem;
        m_GrabberSubsystem = grabberSubsystem;

        // Driver controls.
        dLeftBumper.onTrue(new SetOrientationCmd(m_DriveSubsystem));
        dRightBumper.onTrue(new BrakeCmd(m_DriveSubsystem));
        dA.onTrue(new ChangePositionCmd(m_ExtenderSubsystem));
        dB.onTrue(new ChangePositionCmd(m_FourBarSubsystem));
        dY.onTrue(new RunGrabberCmd(m_GrabberSubsystem));
        dDPadDown.onTrue(new SetGrabberSpeedCmd(m_GrabberSubsystem, Grabber.kDropSpeed));
        dDPadLeft.onTrue(new SetGrabberSpeedCmd(m_GrabberSubsystem, Grabber.kShootSpeed));
        dDPadRight.onTrue(new SetGrabberSpeedCmd(m_GrabberSubsystem, Grabber.kGrabSpeed));
    }

    public XboxController getDriverController() {
        return driverController;
    }

    public XboxController getOperatorController() {
        return operatorController;
    }
}
