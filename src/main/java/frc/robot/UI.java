package frc.robot;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.auto.Trajectories;
import frc.robot.commands.auto.AutoTest1;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ExtenderSubsystem;
import frc.robot.subsystems.FourBarSubsystem;
import frc.robot.subsystems.GrabberSubsystem;

/* Responsible for managing and updating dashboard. */
public class UI extends SubsystemBase {
    // Subsystem references
    private final DriveSubsystem m_DriveSubsystem;
    private final ExtenderSubsystem m_ExtenderSubsystem;
    private final FourBarSubsystem m_FourBarSubsystem;
    private final GrabberSubsystem m_GrabberSubsystem;

    private ShuffleboardTab tab;
    private ShuffleboardLayout driveLayout;
    private SendableChooser<Command> autoChooser;

    public UI(DriveSubsystem driveSubsystem, ExtenderSubsystem extenderSubsystem, 
              FourBarSubsystem fourBarSubsystem, GrabberSubsystem grabberSubsystem, Trajectories paths) {
        m_DriveSubsystem = driveSubsystem;
        m_ExtenderSubsystem = extenderSubsystem;
        m_FourBarSubsystem = fourBarSubsystem;
        m_GrabberSubsystem = grabberSubsystem;

        tab = Shuffleboard.getTab("SmartDashboard");
        driveLayout = tab.getLayout("Drive", BuiltInLayouts.kGrid)
            .withSize(2, 4)
            .withPosition(0, 0);

        autoChooser = new SendableChooser<>();
        autoChooser.setDefaultOption("None", null);
        autoChooser.addOption("Test Routine 1", new AutoTest1(paths, m_DriveSubsystem).getCommand());

        tab.add("Routine Select", autoChooser)
            .withWidget(BuiltInWidgets.kComboBoxChooser)
            .withPosition(7, 1)
            .withSize(2, 1);


        driveLayout.add("Robot Heading", m_DriveSubsystem.getGyro())
            .withWidget(BuiltInWidgets.kGyro)
            .withPosition(0, 0)
            .withSize(2, 2);
    }

    @Override
    public void periodic() {

    }

    public Command getChosenAuto() {
        return autoChooser.getSelected();
    }
}
