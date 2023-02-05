package frc.robot.commands;

import static frc.robot.Constants.Controller;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Range;
import frc.robot.Robot;
import frc.robot.Util;
import frc.robot.subsystems.DriveSubsystem;

public class DriveCmd extends CommandBase {
    private final DriveSubsystem m_DriveSubsystem;
    private Supplier<Double> zSpeedFunc, xSpeedFunc, zRotationFunc;

    public DriveCmd(Supplier<Double> zSpeedFunc, Supplier<Double> xSpeedFunc, Supplier<Double> zRotationFunc) {
        this.m_DriveSubsystem = Robot.m_DriveSubsystem;
        this.zSpeedFunc = zSpeedFunc;
        this.xSpeedFunc = xSpeedFunc;
        this.zRotationFunc = zRotationFunc;
        addRequirements(Robot.m_DriveSubsystem);
    }

    @Override
    public void execute() {
        double zSpeed = zSpeedFunc.get();
        double xSpeed = xSpeedFunc.get();
        double zRotation = zRotationFunc.get();
        
        Range initial = new Range(1.00, Controller.kDeadzone);
        Range limited = new Range(1.00, 0.00);
        
        double newZSpeed = 0.0;
        if (Math.abs(zSpeed) > Controller.kDeadzone - 0.01) {
            newZSpeed = Range.convert(Math.abs(zSpeed), initial, limited);
            newZSpeed = Util.matchSign(zSpeed, newZSpeed);
        }
        
        double newXSpeed = 0.0;
        if (Math.abs(xSpeed) > Controller.kDeadzone - 0.01) {
            newXSpeed = Range.convert(Math.abs(xSpeed), initial, limited);
            newXSpeed = Util.matchSign(xSpeed, newXSpeed);
        }

        double newZRotation = 0.0;
        if (Math.abs(zRotation) > Controller.kDeadzone - 0.01) {
            newZRotation = Range.convert(Math.abs(zRotation), initial, limited);
            newZRotation = Util.matchSign(zRotation, newZRotation);
        }   

        // Y-axis must be flipped because up on the left stick is neg.
        m_DriveSubsystem.drive(-newZSpeed, newXSpeed, newZRotation);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
