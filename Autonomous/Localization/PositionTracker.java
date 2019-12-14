//package org.firstinspires.ftc.teamcode.Autonomous.Localization;
//
//public class PositionTracker {
//
//    private double currentX, currentY, currentAngle; //angle ccw
//    public int leftEncoderTicks, rightEncoderTicks, strafeEncoderTicks;
//    public int lastLeftEncoderTicks, lastRightEncoderTicks, lastStrafeEncoderTicks;
//    //public final double INCHES_PER_TICK = tbd;
//    //public final double width = distance b/w dead wheels;
//
//    public PositionTracker(double initX, double initY, double initAngle){
//        currentX = initX;
//        currentY = initY;
//        currentAngle = initAngle;
//        leftEncoderTicks = 0;
//        rightEncoderTicks = 0;
//        strafeEncoderTicks = 0;
//        lastLeftEncoderTicks = 0;
//        lastRightEncoderTicks = 0;
//        lastStrafeEncoderTicks = 0;
//    }
//
//    public double getCurrentX(){
//        return currentX;
//    }
//
//    public double getCurrentY(){
//        return currentY;
//    }
//
//    public double getCurrentAngle(){
//        return currentAngle;
//    }
//
//    public void updateTicks(){
//        lastLeftEncoderTicks = leftEncoderTicks;
//        lastRightEncoderTicks = rightEncoderTicks;
//        lastStrafeEncoderTicks = strafeEncoderTicks;
//        //leftEncoderTicks = from whatever method;
//        //rightEncoderTicks = from whatever method;
//        //strafeEncoderTicks = from whatever method;
//    }
//
//    public void updateLocationAndPose(String type){
//        double sRight = (rightEncoderTicks - lastRightEncoderTicks)*INCHES_PER_TICK;
//        double sLeft = (leftEncoderTicks - lastLeftEncoderTicks)*INCHES_PER_TICK;
//        double sAvg = (sLeft+sRight)/2;
//        double deltaAngle = (sRight-sLeft)/(width);
//        if(type.equals("strafe")){
//            double sStrafe = strafeEncoderTicks*INCHES_PER_TICK;
//            double strafeX = sAvg*Math.cos(currentAngle) + sStrafe*Math.sin(currentAngle);
//            double strafeY = sAvg*Math.sin(currentAngle) - sStrafe*Math.cos(currentAngle);
//            currentX += strafeX;
//            currentY += strafeY;
//            currentAngle += deltaAngle;
//        }
//        else{
//            double deltaX = sAvg*Math.cos(currentAngle + deltaAngle/2);
//            double deltaY = sAvg*Math.sin(currentAngle + deltaAngle/2);
//            currentX += deltaX;
//            currentY += deltaY;
//            currentAngle += deltaAngle;
//        }
//    }
//}
