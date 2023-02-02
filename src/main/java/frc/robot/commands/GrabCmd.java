package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Grabber;

public class GrabCmd extends CommandBase {
    Grabber grabber;

    public GrabCmd(Grabber grabber) {
        this.grabber = grabber;
        addRequirements(grabber);
    }

    @Override
    public void execute() {
        grabber.grab();
    }

    @Override
    public void end(boolean interrupted) {
        grabber.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
