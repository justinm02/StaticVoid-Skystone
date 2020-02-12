package org.firstinspires.ftc.teamcode.Localization;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class PositionTracker {

    private double currentX, currentY, currentAngle; //angle ccw
    private double initX, initY, initAngle;
    public double leftEncoderTicks, rightEncoderTicks, strafeEncoderTicks;
    public double lastLeftEncoderTicks, lastRightEncoderTicks, lastStrafeEncoderTicks;
    private final double WHEEL_CIRCUMFERENCE_IN = Math.PI*3.05;
    private final double DEADWHEEL_INCHES_OVER_TICKS = WHEEL_CIRCUMFERENCE_IN/4096;
    private final double width = 11.5;

    public PositionTracker(double initX, double initY, double initAngle){
        initX = this.initX;
        initY = this.initY;
        initAngle = this.initAngle;
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

    public double getInitX() {
        return initX;
    }

    public double getInitY() {
        return initY;
    }

    public double getInitAngle() {
        return initAngle;
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

    public void resetPosition() {

    }

    public void updateTicks(double left, double right, double strafe){
        lastLeftEncoderTicks = leftEncoderTicks;
        lastRightEncoderTicks = rightEncoderTicks;
        lastStrafeEncoderTicks = strafeEncoderTicks;

        leftEncoderTicks = left;
        rightEncoderTicks = right;
        strafeEncoderTicks = strafe;
    }

    public void updateLocationAndPose(Telemetry telemetry, String type){
        double sRight = (rightEncoderTicks - lastRightEncoderTicks)*DEADWHEEL_INCHES_OVER_TICKS; //encoder is reversed
        double sLeft = (leftEncoderTicks - lastLeftEncoderTicks)*DEADWHEEL_INCHES_OVER_TICKS;
        double sAvg = (sLeft+sRight)/2;
        double deltaAngle = (sRight-sLeft)/(width);
        double deltaX, deltaY;

        double sStrafe = (strafeEncoderTicks-lastStrafeEncoderTicks)*DEADWHEEL_INCHES_OVER_TICKS; //encoder is reversed


        if(type.equals("spline")){
            deltaX = sAvg*Math.cos(currentAngle + deltaAngle/2);
            deltaY = sAvg*Math.sin(currentAngle + deltaAngle/2);
        }
        else {
            deltaX = sAvg * Math.cos(Math.toRadians(currentAngle)) + sStrafe * Math.sin(Math.toRadians(currentAngle));
            deltaY = sAvg * Math.sin(Math.toRadians(currentAngle)) - sStrafe * Math.cos(Math.toRadians(currentAngle));
        }

        currentX += deltaX;
        currentY += deltaY;
        currentAngle += deltaAngle;

//        telemetry.addData("X sAvg converted",  sAvg * Math.cos(Math.toRadians(currentAngle)));
//        telemetry.addData("sStrafe converted", sStrafe * Math.sin(Math.toRadians(currentAngle)));
//        telemetry.addData("deltaX", deltaX);
//        telemetry.addData("currentAngle", Math.toDegrees(currentAngle));
//        telemetry.addData("sRight", sRight);
//        telemetry.addData("sLeft", sLeft);
//        telemetry.addData("currentX", currentX);
//        telemetry.addData("currentY", currentY);
        telemetry.update();
    }
}