package frc.robot.subsystems;

import frc.robot.Constants.*;
import frc.robot.MotorUtil;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
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

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import com.kauailabs.navx.frc.AHRS;

public class DriveSubsystem extends SubsystemBase {
    /*
     * Motor order is:
     *     Front left,
     *     Front right,
     *     Back left,
     *     Back right
     */
    private WPI_TalonFX[] motors;

    private MecanumDrive driveTrain;
    private MecanumDriveKinematics kinematics;
    private MecanumDriveOdometry odometry;

    private SimpleMotorFeedforward feedForward;
    private PIDController[] drivePID;

    private AHRS gyro;
    private Pose2d pose;

    public double zSpeed;
    public double xSpeed;
    public double zRotation;

    public double prevZSpeed;
    public double prevXSpeed;
    public double prevZRotation;

    private boolean isFieldOriented;
    private boolean isBraking;

    public DriveSubsystem() {
        // Motors are created and configured.
        motors[0] = new WPI_TalonFX(Drive.kFLMotorID);
        motors[1] = new WPI_TalonFX(Drive.kFRMotorID);
        motors[2] = new WPI_TalonFX(Drive.kBLMotorID);
        motors[3] = new WPI_TalonFX(Drive.kBRMotorID);
        configureMotors(motors[0], motors[1], motors[2], motors[3]);
        
        // Odometry and gyro setup.
        gyro = new AHRS(SPI.Port.kMXP);
        resetGyro();
        pose = new Pose2d();

        // Kinematics, odometry, drive PID. Primarily for autonomous.
        kinematics = Drive.kKinematics;
        odometry = new MecanumDriveOdometry(
            kinematics, 
            getHeading(), 
            new MecanumDriveWheelPositions(
                MotorUtil.getMotorDistance(motors[0]), 
                MotorUtil.getMotorDistance(motors[1]),
                MotorUtil.getMotorDistance(motors[2]),
                MotorUtil.getMotorDistance(motors[3])
            )
        );
        feedForward = new SimpleMotorFeedforward(Drive.kS, Drive.kV, Drive.kA);
        drivePID[0] = new PIDController(Drive.kP, Drive.kI, Drive.kD);
        drivePID[1] = new PIDController(Drive.kP, Drive.kI, Drive.kD);
        drivePID[2] = new PIDController(Drive.kP, Drive.kI, Drive.kD);
        drivePID[3] = new PIDController(Drive.kP, Drive.kI, Drive.kD);

        prevZSpeed = 0.0;
        prevXSpeed = 0.0;
        prevZRotation = 0.0;

        isFieldOriented = false;
        isBraking = false;

        // Motors aren't in order because this class' motor order is different.
        driveTrain = new MecanumDrive(motors[0], motors[2], motors[1], motors[3]);
        driveTrain.setSafetyEnabled(false);
    }

    @Override
    public void periodic() {
        // Update pose.
        pose = updatePose();

        // SmartDashboard setup.
        SmartDashboard.putBoolean("Field Oriented?", isFieldOriented);
        SmartDashboard.putNumber("GyroPitch", getPitch());
        SmartDashboard.putData("GyroHeading", gyro);
    }

    // Stops the drive's motors.
    public void stop() {
        drive(0, 0, 0);
    }

    public void drive(double zSpeed, double xSpeed, double zRotation) {
        this.zSpeed = zSpeed;
        this.xSpeed = xSpeed;
        this.zRotation = zRotation;

        // Modify speeds and rotations to slow robot.
        if (isBraking) {
           zSpeed = MathUtil.clamp(prevZSpeed, 0.0, prevZSpeed*Drive.kBrakeRate);
           xSpeed = MathUtil.clamp(prevXSpeed, 0.0, prevXSpeed*Drive.kBrakeRate);
           zRotation = MathUtil.clamp(prevZRotation, 0.0, prevZRotation*Drive.kBrakeRate);
        }

        if (!isFieldOriented) {
            driveTrain.driveCartesian(zSpeed, xSpeed, zRotation);
        } else {
            driveTrain.driveCartesian(zSpeed, xSpeed, zRotation, Rotation2d.fromDegrees(getAngle()));
        }

        this.prevZSpeed = this.zSpeed;
        this.prevXSpeed = this.xSpeed;
        this.prevZRotation = this.zRotation;
    }

    private void configureMotors(WPI_TalonFX FLMotor, WPI_TalonFX FRMotor, WPI_TalonFX BLMotor, WPI_TalonFX BRMotor) {
        // Reset motors' settings to their defaults.
        MotorUtil.setFactory(FLMotor);
        MotorUtil.setFactory(FRMotor);
        MotorUtil.setFactory(BLMotor);
        MotorUtil.setFactory(BRMotor);
        
        // Right motors inverted.
        FLMotor.setInverted(Drive.kLeftInverted);
        FRMotor.setInverted(Drive.kRightInverted);
        BLMotor.setInverted(Drive.kLeftInverted);
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
        motors[0].setVoltage(volts.frontLeftVoltage);
        motors[1].setVoltage(volts.frontRightVoltage);
        motors[3].setVoltage(volts.rearLeftVoltage);
        motors[4].setVoltage(volts.rearRightVoltage);
    }

    public Pose2d updatePose() {
        Rotation2d gyroAngle = getHeading();
        var wheelPositions = new MecanumDriveWheelPositions(
            MotorUtil.getMotorDistance(motors[0]),
            MotorUtil.getMotorDistance(motors[1]),
            MotorUtil.getMotorDistance(motors[2]),
            MotorUtil.getMotorDistance(motors[3])
        );
        return odometry.update(gyroAngle, wheelPositions);
    }

    public Rotation2d getHeading() {
        return Rotation2d.fromDegrees(getAngle());
    }

    public double getAngle() {
        return -gyro.getAngle();
    }

    public double getPitch() {
        return gyro.getPitch();
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

    public void resetOdometry(Pose2d pose) {
        MotorUtil.resetEncoders(motors);
        resetGyro();
        System.out.println("WARNING! " + getHeading());
        odometry.resetPosition(getHeading(), 
            new MecanumDriveWheelPositions(0, 0, 0, 0),
            pose);
    }

    public void setFieldOriented() {
        isFieldOriented = !isFieldOriented;
    }

    public boolean getFieldOriented() {
        return isFieldOriented;
    }

    public void setBraking() {
        isBraking = !isBraking;
    }

    public boolean getBraking() {
        return isBraking;
    }

    public MecanumDriveWheelSpeeds getWheelSpeeds() {
        double flRotationsPerSecond = (double) MotorUtil.getMotorVelocity(motors[0]) / Drive.kEncoderResolution / Drive.kGearRatio * 10;
        double flVelocity = flRotationsPerSecond * 2 * Math.PI * Units.inchesToMeters(Drive.kWheelRadius);

        double frRotationsPerSecond = (double) MotorUtil.getMotorVelocity(motors[1]) / Drive.kEncoderResolution / Drive.kGearRatio * 10;
        double frVelocity = frRotationsPerSecond * 2 * Math.PI * Units.inchesToMeters(Drive.kWheelRadius);

        double blRotationsPerSecond = (double) MotorUtil.getMotorVelocity(motors[2]) / Drive.kEncoderResolution / Drive.kGearRatio * 10;
        double blVelocity = blRotationsPerSecond * 2 * Math.PI * Units.inchesToMeters(Drive.kWheelRadius);

        double brRotationsPerSecond = (double) MotorUtil.getMotorVelocity(motors[3]) / Drive.kEncoderResolution / Drive.kGearRatio * 10;
        double brVelocity = brRotationsPerSecond * 2 * Math.PI * Units.inchesToMeters(Drive.kWheelRadius);

        return new MecanumDriveWheelSpeeds(flVelocity, frVelocity, blVelocity, brVelocity);
    }

    public PIDController getDrivePID(int num) {
        return drivePID[num];
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
}
