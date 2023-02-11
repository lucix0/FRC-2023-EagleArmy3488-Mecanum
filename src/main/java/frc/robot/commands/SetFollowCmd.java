package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class SetFollowCmd extends CommandBase {
    private DriveSubsystem m_DriveSubsystem;

    public SetFollowCmd(DriveSubsystem driveSubsystem) {
        m_DriveSubsystem = driveSubsystem;
        addRequirements(m_DriveSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {  }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        m_DriveSubsystem.setFollow();
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {  }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return true;
    }
}
