package frc.lib;

public class Range {
    private double upper = 0;
    private double lower = 0;
    private double length = 0;
    
    public Range(double upper, double lower) {
        this.upper = upper;
        this.lower = lower;
        length = upper - lower;
    }

    // Converts a number from one range to another.
    public static double convert(double value, Range oldRange, Range newRange) {
        double newValue = (((value - oldRange.lower) * newRange.length) / oldRange.length) + newRange.lower;
        return newValue;
    }

    public void setUpper(double newVal) { this.upper = newVal; }
    public void setLower(double newVal) { this.lower = newVal; }

    public double getUpper() { return this.upper; }
    public double getLower() { return this.lower; }
}
