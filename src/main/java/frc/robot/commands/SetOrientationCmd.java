package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class SetOrientationCmd extends CommandBase {
    private final DriveSubsystem driveTrain;
    private boolean complete = false;

    public SetOrientationCmd(DriveSubsystem driveTrain) {
        this.driveTrain = driveTrain;
        addRequirements(driveTrain);
    }

    @Override
    public void execute() {
        driveTrain.setFieldOriented(!driveTrain.getFieldOriented());
        complete = true;
    }

    @Override
    public boolean isFinished() {
        return complete;
    }
}
