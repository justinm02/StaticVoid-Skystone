package org.firstinspires.ftc.teamcode.Autonomous.MotionProfiling;

public class MotionProfiler {
    private double accelProp; //the percent of the distance spent accelerating MUST BE LESS THAN .5

    public MotionProfiler(double accelProportion) {
        accelProp = accelProportion;
    }

    public double getProfilePower(double propTravelled, double maxPower) {
        double multiplier = maxPower / (2 * Math.pow(accelProp / 2, 2));
        if (propTravelled < accelProp / 2) {
            return multiplier * Math.pow(propTravelled, 2);
        } else if (propTravelled < accelProp) {
            return 2 * multiplier * Math.pow(accelProp / 2, 2) - multiplier * Math.pow(propTravelled - accelProp, 2);
        } else if (propTravelled < 1 - accelProp) {
            return 2 * multiplier * Math.pow(accelProp / 2, 2);
        } else if (propTravelled < 1 - accelProp / 2) {
            return 2 * multiplier * Math.pow(accelProp / 2, 2) - multiplier * Math.pow(propTravelled - (1 - accelProp), 2);
        } else if (propTravelled < 1) {
            return multiplier * Math.pow(propTravelled - 1,  2);
        }
        return 0;
    }
}
