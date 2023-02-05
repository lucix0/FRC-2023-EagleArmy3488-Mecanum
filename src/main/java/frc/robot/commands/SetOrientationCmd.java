package frc.robot.commands;

import frc.robot.subsystems.DriveSubsystem;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class SetOrientationCmd extends CommandBase {
    private DriveSubsystem m_DriveSubsystem;
    private boolean complete = false;

    public SetOrientationCmd(DriveSubsystem driveSubsystem) {
        m_DriveSubsystem = driveSubsystem;
        addRequirements(m_DriveSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {  }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        m_DriveSubsystem.setFieldOriented();
        complete = true;
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {  }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return complete;
    }
}
