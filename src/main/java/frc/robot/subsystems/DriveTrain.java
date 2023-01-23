/*
 *  Currently, this is a very barebones mecanum subsystem. The only functionality here is basic movement
 *  using the left stick for strafing and the right stick for rotating. 
 *  This should work just fine, however if there are any issues with the controls, those can be found
 *  in RobotContainer.java in the classes constructor in the driveTrain.setDefaultCommand method call.
 *  Have fun testing :)
 * 
 *      -S
 */

package frc.robot.subsystems;

import static frc.robot.Constants.*;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.kinematics.MecanumDriveMotorVoltages;
import edu.wpi.first.math.kinematics.MecanumDriveOdometry;
import edu.wpi.first.math.kinematics.MecanumDriveWheelPositions;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveTrain extends SubsystemBase {
    private WPI_TalonFX FLMotor, BLMotor, FRMotor, BRMotor;
    private MecanumDrive driveTrain;
    private MecanumDriveKinematics kinematics;
    private MecanumDriveOdometry odometry;
    private SimpleMotorFeedforward feedForward;

    private PIDController FLPID, BLPID, FRPID, BRPID;

    private AHRS gyro;

    public double zSpeed;
    public double xSpeed;
    public double zRotation;

    public DriveTrain() {
        FLMotor = new WPI_TalonFX(DriveConstants.kFLMotorID);
        BLMotor = new WPI_TalonFX(DriveConstants.kBLMotorID);
        FRMotor = new WPI_TalonFX(DriveConstants.kFRMotorID);
        BRMotor = new WPI_TalonFX(DriveConstants.kBRMotorID);
        configureMotors(FLMotor, BLMotor, FRMotor, BRMotor);

        gyro = new AHRS(SPI.Port.kMXP);

        kinematics = new MecanumDriveKinematics(new Translation2d(-0.292, 0.268), new Translation2d(0.292, 0.268), 
            new Translation2d(-0.292, -0.268), new Translation2d(0.292, -0.268));
        odometry = new MecanumDriveOdometry(kinematics, Rotation2d.fromDegrees(gyro.getAngle()), 
            new MecanumDriveWheelPositions(getFLDistance(), getFRDistance(), getBLDistance(), getBRDistance()));
        feedForward = new SimpleMotorFeedforward(DriveConstants.kS, DriveConstants.kV, DriveConstants.kA);
        
        FLPID = new PIDController(DriveConstants.kP, DriveConstants.kI, DriveConstants.kD);
        BLPID = new PIDController(DriveConstants.kP, DriveConstants.kI, DriveConstants.kD);
        FRPID = new PIDController(DriveConstants.kP, DriveConstants.kI, DriveConstants.kD);
        BRPID = new PIDController(DriveConstants.kP, DriveConstants.kI, DriveConstants.kD);

        driveTrain = new MecanumDrive(FLMotor, BLMotor, FRMotor, BRMotor);
        driveTrain.setSafetyEnabled(false);
    }

    public void mecanumDrive(double zSpeed, double xSpeed, double zRotation) {
        this.zSpeed = zSpeed;
        this.xSpeed = xSpeed;
        this.zRotation = zRotation;
        driveTrain.driveCartesian(zSpeed, xSpeed, zRotation);
    }

    private void configureMotors(WPI_TalonFX FLMotor, WPI_TalonFX BLMotor, WPI_TalonFX FRMotor, WPI_TalonFX BRMotor) {
        // Reset motors' settings to their defaults.
        setFactory(FLMotor);
        setFactory(BLMotor);
        setFactory(FRMotor);
        setFactory(BRMotor);
        
        // Right motors inverted.
        FLMotor.setInverted(DriveConstants.kLeftInverted);
        BLMotor.setInverted(DriveConstants.kLeftInverted);
        FRMotor.setInverted(DriveConstants.kRightInverted);
        BRMotor.setInverted(DriveConstants.kRightInverted);

        // Sets minimum time to accelerate to max speed.
        FLMotor.configOpenloopRamp(DriveConstants.kRampInSec);
        BLMotor.configOpenloopRamp(DriveConstants.kRampInSec);
        FRMotor.configOpenloopRamp(DriveConstants.kRampInSec);
        BRMotor.configOpenloopRamp(DriveConstants.kRampInSec);

        // Prevents robots from gliding stopping accel.
        FLMotor.setNeutralMode(NeutralMode.Brake);
        BLMotor.setNeutralMode(NeutralMode.Brake);
        FRMotor.setNeutralMode(NeutralMode.Brake);
        BRMotor.setNeutralMode(NeutralMode.Brake);
    }

    public void setMotorVolts(MecanumDriveMotorVoltages volts) {
        FLMotor.setVoltage(volts.frontLeftVoltage);
        BLMotor.setVoltage(volts.rearLeftVoltage);
        FRMotor.setVoltage(volts.rearRightVoltage);
        BRMotor.setVoltage(volts.rearRightVoltage);
    }

    public MecanumDriveWheelSpeeds getWheelSpeeds() {
        double flRotationsPerSecond = (double) getFLVelocity() / DriveConstants.kEncoderResolution / DriveConstants.kGearRatio * 10;
        double flVelocity = flRotationsPerSecond * 2 * Math.PI * Units.inchesToMeters(DriveConstants.kWheelRadius);

        double blRotationsPerSecond = (double) getBLVelocity() / DriveConstants.kEncoderResolution / DriveConstants.kGearRatio * 10;
        double blVelocity = blRotationsPerSecond * 2 * Math.PI * Units.inchesToMeters(DriveConstants.kWheelRadius);

        double frRotationsPerSecond = (double) getFRVelocity() / DriveConstants.kEncoderResolution / DriveConstants.kGearRatio * 10;
        double frVelocity = frRotationsPerSecond * 2 * Math.PI * Units.inchesToMeters(DriveConstants.kWheelRadius);

        double brRotationsPerSecond = (double) getBRVelocity() / DriveConstants.kEncoderResolution / DriveConstants.kGearRatio * 10;
        double brVelocity = brRotationsPerSecond * 2 * Math.PI * Units.inchesToMeters(DriveConstants.kWheelRadius);

        return new MecanumDriveWheelSpeeds(flVelocity, frVelocity, blVelocity, brVelocity);
    }

    public double getFLDistance() {
        double flDistance = ((double) getFLEncoderPosition()) / DriveConstants.kEncoderResolution / DriveConstants.kGearRatio * 2 * Math.PI * Units.inchesToMeters(DriveConstants.kWheelRadius);
        return flDistance;
    }

    public double getBLDistance() {
        double blDistance = ((double) getBLEncoderPosition()) / DriveConstants.kEncoderResolution / DriveConstants.kGearRatio * 2 * Math.PI * Units.inchesToMeters(DriveConstants.kWheelRadius);
        return blDistance;
    }

    public double getFRDistance() {
        double frDistance = ((double) getFREncoderPosition()) / DriveConstants.kEncoderResolution / DriveConstants.kGearRatio * 2 * Math.PI * Units.inchesToMeters(DriveConstants.kWheelRadius);
        return frDistance;
    }

    public double getBRDistance() {
        double brDistance = ((double) getBREncoderPosition()) / DriveConstants.kEncoderResolution / DriveConstants.kGearRatio * 2 * Math.PI * Units.inchesToMeters(DriveConstants.kWheelRadius);
        return brDistance;
    }

    public double getFLVelocity() {
        return FLMotor.getSelectedSensorVelocity();
    }

    public double getBLVelocity() {
        return BLMotor.getSelectedSensorVelocity();
    }

    public double getFRVelocity() {
        return FRMotor.getSelectedSensorVelocity();
    }

    public double getBRVelocity() {
        return BRMotor.getSelectedSensorVelocity();
    }

    public double getFLEncoderPosition() {
        return FLMotor.getSelectedSensorPosition();
    }

    public double getBLEncoderPosition() {
        return BLMotor.getSelectedSensorPosition();
    }

    public double getFREncoderPosition() {
        return FRMotor.getSelectedSensorPosition();
    }

    public double getBREncoderPosition() {
        return BRMotor.getSelectedSensorPosition();
    }

    public PIDController getFLPID() {
        return FLPID;
    }

    public PIDController getBLPID() {
        return BLPID;
    }

    public PIDController getFRPID() {
        return FRPID;
    }

    public PIDController getBRPID() {
        return BRPID;
    }

    public MecanumDriveKinematics getKinematics() {
        return kinematics;
    }

    public MecanumDriveOdometry getOdometry() {
        return odometry;
    }

    public SimpleMotorFeedforward getFeedForward() {
        return feedForward;
    }

    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }

    public void resetGyro() {
        gyro.reset();
    }

    public void resetEncoders() {
        FLMotor.setSelectedSensorPosition(0);
        BLMotor.setSelectedSensorPosition(0);
        FRMotor.setSelectedSensorPosition(0);
        BRMotor.setSelectedSensorPosition(0);
    }

    public void setFactory(WPI_TalonFX motor) {
        motor.configFactoryDefault();
    }
}
