package frc.robot;

public class Util {
    // Sets sign of one value to match another
    public static double matchSign(double match, double change) {
        change = Math.abs(change);
        if (match < 0) {
            change = -change;
        }
        return change;
    }

    
}
