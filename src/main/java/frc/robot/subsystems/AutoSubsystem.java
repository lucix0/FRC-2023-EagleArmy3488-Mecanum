package frc.robot.subsystems;

import frc.robot.Constants.*;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AutoSubsystem extends SubsystemBase {
    private DriveSubsystem m_DriveSubsystem;

    private DifferentialDriveKinematics kinematics;
    private DifferentialDriveOdometry odometry;

    public AutoSubsystem(DriveSubsystem driveSubsystem) {
        m_DriveSubsystem = driveSubsystem;
        this.kinematics = new DifferentialDriveKinematics(0);
        this.odometry = new DifferentialDriveOdometry(null, 0, 0)
    }
}
