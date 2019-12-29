package org.firstinspires.ftc.teamcode.Localization;

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

    public void updateLocationAndPose(String type){
        double sRight = (rightEncoderTicks - lastRightEncoderTicks)*DEADWHEEL_INCHES_OVER_TICKS;
        double sLeft = (leftEncoderTicks - lastLeftEncoderTicks)*DEADWHEEL_INCHES_OVER_TICKS;
        double sAvg = (sLeft+sRight)/2;
        double deltaAngle = (sRight-sLeft)/(width);
        if(type.equals("strafe")){
            double sStrafe = strafeEncoderTicks*DEADWHEEL_INCHES_OVER_TICKS;
            double strafeX = sAvg*Math.cos(currentAngle) + sStrafe*Math.sin(currentAngle);
            double strafeY = sAvg*Math.sin(currentAngle) - sStrafe*Math.cos(currentAngle);
            currentX += strafeX;
            currentY += strafeY;
            currentAngle += deltaAngle;
        }
        else {
            double deltaX = sAvg*Math.cos(currentAngle + deltaAngle/2);
            double deltaY = sAvg*Math.sin(currentAngle + deltaAngle/2);
            currentX += deltaX;
            currentY += deltaY;
            currentAngle += deltaAngle;
        }
    }
}