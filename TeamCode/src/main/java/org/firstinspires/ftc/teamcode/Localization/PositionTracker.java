package org.firstinspires.ftc.teamcode.Localization;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class PositionTracker {

    private double currentX, currentY, currentAngle; //angle ccw
    public double leftEncoderTicks, rightEncoderTicks, strafeEncoderTicks;
    public double lastLeftEncoderTicks, lastRightEncoderTicks, lastStrafeEncoderTicks;
    private final double WHEEL_CIRCUMFERENCE_IN = Math.PI*3.05;
    private final double DEADWHEEL_INCHES_OVER_TICKS = WHEEL_CIRCUMFERENCE_IN/4096;
    private final double width = 11.5;

    public PositionTracker(double initX, double initY, double initAngle){
        currentX = initX;
        currentY = initY;
        currentAngle = initAngle;
        leftEncoderTicks = 0;
        rightEncoderTicks = 0;
        strafeEncoderTicks = 0;
        lastLeftEncoderTicks = 0;
        lastRightEncoderTicks = 0;
        lastStrafeEncoderTicks = 0;
    }

    public double getCurrentX(){
        return currentX;
    }

    public double getCurrentY(){
        return currentY;
    }

    public double getCurrentAngle(){
        return currentAngle;
    }

    public void updateTicks(double left, double right, double strafe){
        lastLeftEncoderTicks = leftEncoderTicks;
        lastRightEncoderTicks = rightEncoderTicks;
        lastStrafeEncoderTicks = strafeEncoderTicks;

        leftEncoderTicks = left;
        rightEncoderTicks = right;
        strafeEncoderTicks = strafe;
    }

    public void updateLocationAndPose(Telemetry telemetry, double gyroAngle){
        double sRight = (rightEncoderTicks - lastRightEncoderTicks)*DEADWHEEL_INCHES_OVER_TICKS;
        double sLeft = (leftEncoderTicks - lastLeftEncoderTicks)*DEADWHEEL_INCHES_OVER_TICKS;
        double sAvg = (sLeft+sRight)/2;
        double deltaAngle = (sRight-sLeft)/(width);

        double sStrafe = (strafeEncoderTicks-lastStrafeEncoderTicks)*DEADWHEEL_INCHES_OVER_TICKS;

        currentAngle += deltaAngle;

        double deltaX = sAvg*Math.cos(Math.toRadians(currentAngle)) + sStrafe*Math.sin(Math.toRadians(currentAngle));
        double deltaY = sAvg*Math.sin(Math.toRadians(currentAngle)) - sStrafe*Math.cos(Math.toRadians(currentAngle));

        currentX += deltaX;
        currentY += deltaY;

        telemetry.addData("sAvg", sAvg);
        telemetry.addData("sStrafe", sStrafe);
        telemetry.addData("currentAngle", Math.toDegrees(currentAngle));
        telemetry.addData("sRight", sRight);
        telemetry.addData("sLeft", sLeft);
        telemetry.addData("currentX", currentX);
        telemetry.addData("currentY", currentY);
        telemetry.update();
    }
}