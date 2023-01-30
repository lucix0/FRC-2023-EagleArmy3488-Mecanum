package frc.robot.subsystems;

import frc.robot.Constants.*;

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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveTrain extends SubsystemBase {
    private WPI_TalonFX FLMotor, BLMotor, FRMotor, BRMotor;

    private MecanumDrive driveTrain;
    private MecanumDriveKinematics kinematics;
    private MecanumDriveOdometry odometry;

    private SimpleMotorFeedforward feedForward;
    private PIDController FLPID, BLPID, FRPID, BRPID;

    private AHRS gyro;
    private Pose2d pose;

    public double zSpeed;
    public double xSpeed;
    public double zRotation;

    private boolean isFieldOriented;
    private boolean isBraking;

    public DriveTrain() {
        FLMotor = new WPI_TalonFX(Drive.kFLMotorID);
        BLMotor = new WPI_TalonFX(Drive.kBLMotorID);
        FRMotor = new WPI_TalonFX(Drive.kFRMotorID);
        BRMotor = new WPI_TalonFX(Drive.kBRMotorID);
        configureMotors(FLMotor, BLMotor, FRMotor, BRMotor);

        gyro = new AHRS(SPI.Port.kMXP);
        resetGyro();
        pose = new Pose2d();

        kinematics = new MecanumDriveKinematics(new Translation2d(-0.292, 0.268), new Translation2d(0.292, 0.268), 
            new Translation2d(-0.292, -0.268), new Translation2d(0.292, -0.268));
        odometry = new MecanumDriveOdometry(kinematics, Rotation2d.fromDegrees(-gyro.getAngle()), 
            new MecanumDriveWheelPositions(getFLDistance(), getFRDistance(), getBLDistance(), getBRDistance()));
        feedForward = new SimpleMotorFeedforward(Drive.kS, Drive.kV, Drive.kA);
        
        FLPID = new PIDController(Drive.kP, Drive.kI, Drive.kD);
        BLPID = new PIDController(Drive.kP, Drive.kI, Drive.kD);
        FRPID = new PIDController(Drive.kP, Drive.kI, Drive.kD);
        BRPID = new PIDController(Drive.kP, Drive.kI, Drive.kD);

        isFieldOriented = false;

        driveTrain = new MecanumDrive(FLMotor, BLMotor, FRMotor, BRMotor);
        driveTrain.setSafetyEnabled(false);
    }

    @Override
    public void periodic() {
        // Update pose.
        var wheelPositions = new MecanumDriveWheelPositions(getFLDistance(), getFRDistance(), getBLDistance(), getBRDistance());
        Rotation2d gyroAngle = Rotation2d.fromDegrees(-gyro.getAngle());
        pose = odometry.update(gyroAngle, wheelPositions);

        SmartDashboard.putBoolean("Field Oriented", isFieldOriented);
        SmartDashboard.putNumber("Gyro Pitch", gyro.getPitch());
    }

    public void mecanumDrive(double zSpeed, double xSpeed, double zRotation) {
        this.zSpeed = zSpeed;
        this.xSpeed = xSpeed;
        this.zRotation = zRotation;

        if (!isFieldOriented) {
            driveTrain.driveCartesian(zSpeed, xSpeed, zRotation);
        } else {
            driveTrain.driveCartesian(zSpeed, xSpeed, zRotation, Rotation2d.fromDegrees(gyro.getAngle()));
        }
    }

    private void configureMotors(WPI_TalonFX FLMotor, WPI_TalonFX BLMotor, WPI_TalonFX FRMotor, WPI_TalonFX BRMotor) {
        // Reset motors' settings to their defaults.
        setFactory(FLMotor);
        setFactory(BLMotor);
        setFactory(FRMotor);
        setFactory(BRMotor);
        
        // Right motors inverted.
        FLMotor.setInverted(Drive.kLeftInverted);
        BLMotor.setInverted(Drive.kLeftInverted);
        FRMotor.setInverted(Drive.kRightInverted);
        BRMotor.setInverted(Drive.kRightInverted);

        // Sets minimum time to accelerate to max speed.
        FLMotor.configOpenloopRamp(Drive.kRampInSec);
        BLMotor.configOpenloopRamp(Drive.kRampInSec);
        FRMotor.configOpenloopRamp(Drive.kRampInSec);
        BRMotor.configOpenloopRamp(Drive.kRampInSec);

        // Prevents robots from gliding stopping accel.
        FLMotor.setNeutralMode(NeutralMode.Brake);
        BLMotor.setNeutralMode(NeutralMode.Brake);
        FRMotor.setNeutralMode(NeutralMode.Brake);
        BRMotor.setNeutralMode(NeutralMode.Brake);
    }

    public void setMotorVolts(MecanumDriveMotorVoltages volts) {
        FLMotor.setVoltage(volts.frontLeftVoltage);
        BLMotor.setVoltage(volts.rearLeftVoltage);
        FRMotor.setVoltage(volts.frontRightVoltage);
        BRMotor.setVoltage(volts.rearRightVoltage);
    }

    public Rotation2d getHeading() {
        return Rotation2d.fromDegrees(-gyro.getAngle());
    }

    // Reset Functions
    public void resetOdometry(Pose2d pose) {
        resetEncoders();
        resetGyro();
        System.out.println("WARNING! " + getHeading());
        odometry.resetPosition(getHeading(), 
            new MecanumDriveWheelPositions(0, 0, 0, 0),
            pose);
    }

    public void setFieldOriented(boolean bool) {
        isFieldOriented = bool;
    }

    public boolean getFieldOriented() {
        return isFieldOriented;
    }

    public void setBraking(boolean bool) {
        isBraking = bool;
    }

    public boolean getBraking() {
        return isBraking;
    }

    public MecanumDriveWheelSpeeds getWheelSpeeds() {
        double flRotationsPerSecond = (double) getFLVelocity() / Drive.kEncoderResolution / Drive.kGearRatio * 10;
        double flVelocity = flRotationsPerSecond * 2 * Math.PI * Units.inchesToMeters(Drive.kWheelRadius);

        double blRotationsPerSecond = (double) getBLVelocity() / Drive.kEncoderResolution / Drive.kGearRatio * 10;
        double blVelocity = blRotationsPerSecond * 2 * Math.PI * Units.inchesToMeters(Drive.kWheelRadius);

        double frRotationsPerSecond = (double) getFRVelocity() / Drive.kEncoderResolution / Drive.kGearRatio * 10;
        double frVelocity = frRotationsPerSecond * 2 * Math.PI * Units.inchesToMeters(Drive.kWheelRadius);

        double brRotationsPerSecond = (double) getBRVelocity() / Drive.kEncoderResolution / Drive.kGearRatio * 10;
        double brVelocity = brRotationsPerSecond * 2 * Math.PI * Units.inchesToMeters(Drive.kWheelRadius);

        return new MecanumDriveWheelSpeeds(flVelocity, frVelocity, blVelocity, brVelocity);
    }

    public double getFLDistance() {
        double flDistance = ((double) getFLEncoderPosition()) / Drive.kEncoderResolution / Drive.kGearRatio * 2 * Math.PI * Units.inchesToMeters(Drive.kWheelRadius);
        return flDistance;
    }

    public double getBLDistance() {
        double blDistance = ((double) getBLEncoderPosition()) / Drive.kEncoderResolution / Drive.kGearRatio * 2 * Math.PI * Units.inchesToMeters(Drive.kWheelRadius);
        return blDistance;
    }

    public double getFRDistance() {
        double frDistance = ((double) getFREncoderPosition()) / Drive.kEncoderResolution / Drive.kGearRatio * 2 * Math.PI * Units.inchesToMeters(Drive.kWheelRadius);
        return frDistance;
    }

    public double getBRDistance() {
        double brDistance = ((double) getBREncoderPosition()) / Drive.kEncoderResolution / Drive.kGearRatio * 2 * Math.PI * Units.inchesToMeters(Drive.kWheelRadius);
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

    public AHRS getGyro() {
        return gyro;
    }

    public Pose2d getPose() {
        return pose;
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
