package frc.robot;

/* All ids and ports are stored here. */
public abstract class RobotMap {
    // Controllers
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 1;

    // Drive subsystem
    public static final int kFLMotorID = 6;
    public static final int kFRMotorID = 5;
    public static final int kBLMotorID = 8;
    public static final int kBRMotorID = 7;

    // Extender
    public static final int kExtenderMotorID1 = 3;
    public static final int kExtenderMotorID2 = 2;
    
    // FourBar
    public static final int kFourBarMotorID1 = 1;
    public static final int kFourBarMotorID2 = 0;

    // Grabber
    public static final int kGrabberMotorID = 4;
}
