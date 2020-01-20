package org.firstinspires.ftc.teamcode.MotionProfiling;

public class MotionProfiler {
    private double accelProp; //the percent of the distance spent accelerating MUST BE LESS THAN .5

    public MotionProfiler(double accelProportion) {
        accelProp = accelProportion;
    }

    public double getProfilePower(double propTravelled, double maxPower, double initPower, double finPower) {
        double accelMult = (maxPower - initPower) / (2 * Math.pow(accelProp / 2, 2));
        double decelMult = (maxPower - finPower) / (2 * Math.pow(accelProp / 2, 2));
        if (propTravelled < accelProp / 2) {
            return accelMult * Math.pow(propTravelled, 2) + initPower;
        } else if (propTravelled < accelProp) {
            return maxPower - accelMult * Math.pow(propTravelled - accelProp, 2);
        } else if (propTravelled < 1 - accelProp) {
            return maxPower;
        } else if (propTravelled < 1 - accelProp / 2) {
            return maxPower - decelMult * Math.pow(propTravelled - (1 - accelProp), 2);
        } else if (propTravelled < 1) {
            return decelMult * Math.pow(propTravelled - 1, 2) + finPower;
        }
        return finPower;
    }
}
