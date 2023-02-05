package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.subsystems.DriveSubsystem;

public class SetOrientationCmd extends CommandBase {
    private final DriveSubsystem m_DriveSubsystem;
    private boolean complete = false;

    public SetOrientationCmd() {
        this.m_DriveSubsystem = Robot.m_DriveSubsystem;
        addRequirements(m_DriveSubsystem);
    }

    @Override
    public void execute() {
        m_DriveSubsystem.setFieldOriented(!m_DriveSubsystem.getFieldOriented());
        complete = true;
    }

    @Override
    public boolean isFinished() {
        return complete;
    }
}
