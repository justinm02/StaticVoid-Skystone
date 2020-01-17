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

    public void updateLocationAndPose(String type, Telemetry telemetry, double gyroAngle){
        double sRight = (rightEncoderTicks - lastRightEncoderTicks)*DEADWHEEL_INCHES_OVER_TICKS;
        double sLeft = (leftEncoderTicks - lastLeftEncoderTicks)*DEADWHEEL_INCHES_OVER_TICKS;
        double sAvg = (sLeft+sRight)/2;
        double deltaAngle = (sRight-sLeft)/(width);
        //if(type.equals("strafe")){
            double sStrafe = strafeEncoderTicks*DEADWHEEL_INCHES_OVER_TICKS;
            /*double strafeX = sAvg*Math.cos(currentAngle) + sStrafe*Math.sin(currentAngle);
            double strafeY = sAvg*Math.sin(currentAngle) - sStrafe*Math.cos(currentAngle);*/
            //double strafeMagnitude = Math.sqrt(Math.pow(sAvg, 2) + Math.pow(sStrafe, 2));
            currentAngle += deltaAngle;

            double deltaX = sAvg*Math.cos(Math.toRadians(gyroAngle)) + sStrafe*Math.sin(Math.toRadians(gyroAngle));
            double deltaY = sAvg*Math.sin(Math.toRadians(gyroAngle)) - sStrafe*Math.cos(Math.toRadians(gyroAngle));

            currentX += deltaX;
            currentY += deltaY;
        /*}
        else {
            double deltaX = sAvg*Math.cos(currentAngle + deltaAngle/2);
            double deltaY = sAvg*Math.sin(currentAngle + deltaAngle/2);

            double deltaX = sAvg*Math.cos(Math.toRadians(gyroAngle));
            double deltaY = sAvg*Math.sin(Math.toRadians(gyroAngle));

            currentX += deltaX;
            currentY += deltaY;
            currentAngle += deltaAngle;
        }*/
        telemetry.addData("sAvg", sAvg);
        telemetry.addData("deltaAngle", Math.toDegrees(gyroAngle + deltaAngle/2));
        telemetry.addData("currentX", currentX);
        telemetry.addData("currentY", currentY);
        telemetry.update();
    }
}