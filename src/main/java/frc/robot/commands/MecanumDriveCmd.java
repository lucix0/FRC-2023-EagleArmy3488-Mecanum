package frc.robot.commands;

import static frc.robot.Constants.ControllerConstants;

import java.util.function.Supplier;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Range;
import frc.robot.Util;
import frc.robot.subsystems.DriveTrain;

public class MecanumDriveCmd extends CommandBase {
    private final DriveTrain driveTrain;
    private Supplier<Double> zSpeedFunc, xSpeedFunc, zRotationFunc;

    public MecanumDriveCmd(DriveTrain driveTrain, Supplier<Double> zSpeedFunc, Supplier<Double> xSpeedFunc, Supplier<Double> zRotationFunc) {
        this.driveTrain = driveTrain;
        this.zSpeedFunc = zSpeedFunc;
        this.xSpeedFunc = xSpeedFunc;
        this.zRotationFunc = zRotationFunc;
        addRequirements(driveTrain);
    }

    @Override
    public void execute() {
        double zSpeed = zSpeedFunc.get();
        double xSpeed = xSpeedFunc.get();
        double zRotation = zRotationFunc.get();

        // Restricts values between the deadzone and 1.00
        Range initial = new Range(1.00, 0.00);
        Range limited = new Range(1.00, ControllerConstants.kStickDeadzone);

        double newZSpeed = Range.convert(zSpeed, initial, limited);
        newZSpeed = Util.matchSign(zSpeed, newZSpeed);

        double newXSpeed = Range.convert(xSpeed, initial, limited);
        newXSpeed = Util.matchSign(xSpeed, newXSpeed);

        double newZRotation = Range.convert(zRotation, initial, limited);
        newZRotation = Util.matchSign(zRotation, newZRotation);

        // Y-axis must be flipped because up on the left stick is neg.
        driveTrain.mecanumDrive(-newZSpeed, newXSpeed, newZRotation);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
