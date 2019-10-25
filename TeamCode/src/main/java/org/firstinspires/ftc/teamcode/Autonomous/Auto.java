package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.teamcode.Autonomous.PID.PID;
import org.firstinspires.ftc.teamcode.Autonomous.Spline.Bezier;
import org.firstinspires.ftc.teamcode.Autonomous.Spline.Waypoint;
import org.firstinspires.ftc.teamcode.Autonomous.Vuforia.Vuforia;

import com.qualcomm.robotcore.util.Range;

public abstract class Auto extends LinearOpMode {
    private ElapsedTime runtime;
    private DcMotorEx leftFront, leftBack, rightFront, rightBack, slide, parallelEncoderTracker, perpendicularEncoderTracker;
    private DcMotorEx[] motors;
    private Servo frontPlatformLatcher, backPlatformLatcher, leftClaw, rightClaw;
    private BNO055IMU imu;
    private PID ForwardHeadingPid = new PID(0.0175, 0, 0.005);
    private PID BackwardsHeadingPID = new PID(0, 0, 0);
    private PID strafePID = new PID(.05, 0, 0); //still need to be determined and tuned
    public int initialParallelEncoderPosition;
    public int initialPerpendicularEncoderPosition;
    private double xPos = 0;
    private double yPos = 0;
    private final int parallelDeadWheelTicks = 1320; //tested and validated ticks per revolution of parallel deadwheel; old: 1440
    private final int perpendicularDeadWheelTicks = 720; //tested and validated ticks per revolution of parallel deadwheel; old: 1440
    private final double WHEEL_CIRCUMFERENCE_IN = Math.PI*3.05; //circumference of parallel deadwheel
    private final double PARALLEL_INCHES_OVER_TICKS = WHEEL_CIRCUMFERENCE_IN/parallelDeadWheelTicks; //conversion used to convert inches to ticks (what encoderTracker.getCurrentPosition() reads)
    private final double PERPENDICULAR_INCHES_OVER_TICKS = WHEEL_CIRCUMFERENCE_IN/perpendicularDeadWheelTicks; //conversion used to convert inches to tick

    private Vuforia vuforia = new Vuforia();
    private int cameraMonitorViewId;
    private VuforiaLocalizer.Parameters parameters;

    enum SKYSTONE_POSITION {
        LEFT,
        MIDDLE,
        RIGHT
    }

    public void initialize() {
        initMotors();
        initServos();
        initGyro();
        initVuforia();

        runtime = new ElapsedTime();

        xPos = 0;
        yPos = 0;

        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    public void initMotors() {
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftBack = hardwareMap.get(DcMotorEx.class, "leftBack");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        rightBack = hardwareMap.get(DcMotorEx.class, "rightBack");
        parallelEncoderTracker = hardwareMap.get(DcMotorEx.class, "parallelEncoderTracker");
        slide = hardwareMap.get(DcMotorEx.class, "slide");

        motors = new DcMotorEx[] {leftFront, leftBack, rightFront, rightBack};

        for (DcMotorEx motor : motors) {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        perpendicularEncoderTracker = hardwareMap.get(DcMotorEx.class, "perpendicularEncoderTracker");

        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        //parallelEncoderTracker.setDirection(DcMotor.Direction.REVERSE);
        //perpendicularEncoderTracker.setDirection(DcMotor.Direction.REVERSE);



        parallelEncoderTracker.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        parallelEncoderTracker.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        perpendicularEncoderTracker.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        perpendicularEncoderTracker.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void initServos() {
        frontPlatformLatcher = hardwareMap.servo.get("frontLatcher");
        backPlatformLatcher = hardwareMap.servo.get("backLatcher");

        leftClaw = hardwareMap.servo.get("leftClaw");
        rightClaw = hardwareMap.servo.get("rightClaw");
    }

    public void initGyro() {
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imu.initialize(parameters);
    }

    public void initVuforia() {
        cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        vuforia.initVuforia(parameters);
    }

    public SKYSTONE_POSITION determineSkystonePlacement() throws InterruptedException {
        double startingPosition = perpendicularEncoderTracker.getCurrentPosition();
        double yPosition = 0;
        double inches = 22;

        vuforia.targetsSkyStone.activate();
        pause(0.5);
        while (!vuforia.targetVisible && Math.abs(perpendicularEncoderTracker.getCurrentPosition()-startingPosition) < (int)(inches/PERPENDICULAR_INCHES_OVER_TICKS)) {
            heartbeat();
            correction(.3, 0, "straferight", false);
            yPosition = vuforia.getYPosition();
            telemetry.addData("diff", Math.abs(perpendicularEncoderTracker.getCurrentPosition()-startingPosition)*PERPENDICULAR_INCHES_OVER_TICKS);
            telemetry.addData("perpendicular", perpendicularEncoderTracker.getCurrentPosition());
            telemetry.update();
        }
        vuforia.targetsSkyStone.deactivate();

        halt();

        double current = runtime.time();
        while (Math.abs(current - runtime.time()) < 2) {
            telemetry.addData("yPosition", yPosition);
            telemetry.addData("diff", Math.abs(perpendicularEncoderTracker.getCurrentPosition()-startingPosition)*PERPENDICULAR_INCHES_OVER_TICKS);
            telemetry.addData("perpendicular", perpendicularEncoderTracker.getCurrentPosition());
            telemetry.update();
        }

        if (Math.abs(perpendicularEncoderTracker.getCurrentPosition()-startingPosition) >= (int)(inches/PERPENDICULAR_INCHES_OVER_TICKS) || yPosition > 6)
            return SKYSTONE_POSITION.RIGHT;
        else if (yPosition < -4)
            return SKYSTONE_POSITION.LEFT;
        return SKYSTONE_POSITION.MIDDLE;
    }

    public void move(double targetHeading, double power, int distance, String direction) throws InterruptedException {
        double startingPosition = parallelEncoderTracker.getCurrentPosition();
        boolean inverted = false;

        if (power < 0)
            inverted = true;

        int target = (int)(distance/PARALLEL_INCHES_OVER_TICKS); //how many ticks robot will travel

        while (Math.abs(startingPosition - parallelEncoderTracker.getCurrentPosition()) < target) {
            correction(power, targetHeading, direction, inverted);
            telemetry.addData("diff", Math.abs(parallelEncoderTracker.getCurrentPosition()-startingPosition)*PARALLEL_INCHES_OVER_TICKS);
            telemetry.addData("parallel", parallelEncoderTracker.getCurrentPosition());
            telemetry.update();
            //trackPosition();
            heartbeat();
        }

        halt();
    }

    public void moveSlideByTime() {
        double basePosition = slide.getCurrentPosition();

        while (Math.abs(basePosition - slide.getCurrentPosition()) < 300) {
            slide.setPower(-.2);
        }
        slide.setPower(0);
    }

    public void strafe(double power, int targetHeading, String direction, double inches) throws InterruptedException {
    int ticks = (int)(inches/PERPENDICULAR_INCHES_OVER_TICKS);
        double startingPosition = perpendicularEncoderTracker.getCurrentPosition();

        while (Math.abs(perpendicularEncoderTracker.getCurrentPosition() - startingPosition) < ticks) {
            correction(power, targetHeading, direction, false);
            heartbeat();
            telemetry.addData("diff", Math.abs(perpendicularEncoderTracker.getCurrentPosition()-startingPosition)*PERPENDICULAR_INCHES_OVER_TICKS);
            telemetry.addData("perpendicular", perpendicularEncoderTracker.getCurrentPosition());
            telemetry.update();
        }
        halt();
    }

    public void splineMove(double[] xcoords, double[] ycoords, double power, double offset) throws InterruptedException {
        double startingPosition = parallelEncoderTracker.getCurrentPosition();
        Waypoint[] coords = new Waypoint[xcoords.length];
        boolean inverted = false;

        //set Waypoints per each (x,y)
        for (int i = 0; i < coords.length; i++) {
            coords[i] = new Waypoint(xcoords[i], ycoords[i]);
        }

        //if spline backwards, set inverted to true (let's correction method know to make adjustments to targetHeading in PD correction method)
        if (power < 0) {
            inverted = true;
        }

        //sets new spline, defines important characteristics
        Bezier spline = new Bezier(coords);
        double t = 0;
        double distanceTraveled;
        double arclength = spline.getArcLength(); //computes arc length by adding infinitesimally small slices of sqrt( (dx/dt)^2 + (dy/dt)^2 ) (distance formula). This method uses integration, a fundamental component in calculus
        int lastAngle = (int)currentAngle();
        while (t<=1.0) {
            //trackPosition();
            heartbeat();
            //constantly adjusts heading based on what the current spline angle should be based on the calculated t
            correction(power, (int)(180/Math.PI*spline.getAngle(t, Math.PI/180*lastAngle, offset)), "spline", inverted); //converts lastAngle to radians
            lastAngle = (int)(180/Math.PI*spline.getAngle(t, Math.PI/180*lastAngle, offset)); //converts lastAngle to degrees for telemetry
            //distanceTraveled computed by converting encoderTraveled ticks on deadwheel to inches traveled
            distanceTraveled = PARALLEL_INCHES_OVER_TICKS*(parallelEncoderTracker.getCurrentPosition()-startingPosition);
            //t measures progress along curve. Very important for computing splineAngle.
            t = Math.abs(distanceTraveled/arclength);
            telemetry.addData("Distance ", distanceTraveled);
            telemetry.addData("t ", t);
            telemetry.addData("error", currentAngle() - lastAngle);
            telemetry.addData("dxdt", spline.getdxdt(t));
            telemetry.addData("spline angle", (int)(180/Math.PI*spline.getAngle(t, Math.PI/180*lastAngle, offset)));
            telemetry.addData("current angle", currentAngle());
            telemetry.update();
        }

        halt();
    }

    public void turn(double heading, double power, String direction) throws InterruptedException {
        while (Math.abs(heading - currentAngle()) < 5) {
            turn(direction, power);
            heartbeat();
        }
    }

    public void turn(String direction, double power) {
        if (direction.equals("right")) {
            leftFront.setPower(power);
            leftBack.setPower(power);
            rightFront.setPower(-power);
            rightBack.setPower(-power);
        }
        else if (direction.equals("left")) {
            leftFront.setPower(-power);
            leftBack.setPower(-power);
            rightFront.setPower(power);
            rightBack.setPower(power);
        }
    }

    public double currentAngle() {
        //returns heading from gyro using unit circle values (-180 to 180 degrees, -pi to pi radians. We're using degrees)
        return imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
    }

    public void correction(double power, double targetHeading, String movementType, boolean inverted)
    {
        //sets target and current angles
        double target = targetHeading;
        double current = currentAngle();

        //if the spline motion is backwards, the target must be flipped 180 degrees in order to match with spline.getAngle()
        if (inverted && movementType.contains("spline")) {
            target = targetHeading + 180;
        }

        //when axis between -179 and 179 degrees is crossed, degrees must be converted from 0 - 360 degrees. 179-(-179) = 358. 179 - 181 = -2. Big difference
        if (targetHeading < -135 && currentAngle() > 135) {
            target = targetHeading + 360.0;
        }
        else if (targetHeading > 135 && currentAngle() < -135) {
            current = currentAngle() + 360.0;
        }

        //PD correction for both regular and spline motion
        if (movementType.contains("straight") || movementType.contains("spline")) {
            leftFront.setPower(Range.clip(power + ForwardHeadingPid.getCorrection(current - target, runtime), -1.0, 1.0));
            rightFront.setPower(Range.clip(power - ForwardHeadingPid.getCorrection(current - target, runtime), -1.0, 1.0));
            leftBack.setPower(Range.clip(power + ForwardHeadingPid.getCorrection(current - target, runtime), -1.0, 1.0));
            rightBack.setPower(Range.clip(power - ForwardHeadingPid.getCorrection(current - target, runtime), -1.0, 1.0));
        }

        //pd correction for strafe motion. Right and left are opposites
        else if (movementType.contains("strafe")) {
            if (movementType.contains("left")) {
                leftFront.setPower(Range.clip(-power - strafePID.getCorrection(current - target, runtime), -1.0, 1.0));
                rightFront.setPower(Range.clip(power/* - strafePID.getCorrection(current - target, runtime)*/, -1.0, 1.0));
                leftBack.setPower(Range.clip(power/* + strafePID.getCorrection(current - target, runtime)*/, -1.0, 1.0));
                rightBack.setPower(Range.clip(-power - strafePID.getCorrection(current - target, runtime), -1.0, 1.0));
            }
            else if (movementType.contains("right")) {
                leftFront.setPower(Range.clip(power/* + strafePID.getCorrection(current - target, runtime)*/, -1.0, 1.0));
                rightFront.setPower(Range.clip(-power/* + strafePID.getCorrection(current - target, runtime)*/, -1.0, 1.0));
                leftBack.setPower(Range.clip(-power /*- strafePID.getCorrection(current - target, runtime)*/, -1.0, 1.0));
                rightBack.setPower(Range.clip(power - strafePID.getCorrection(current - target, runtime), -1.0, 1.0));
            }
        }

        telemetry.addData("angle", current);
        telemetry.addData("error", target-current);
        telemetry.addData("power BR", rightBack.getPower());
        telemetry.addData("power BL", leftBack.getPower());
        telemetry.addData("power FR", rightFront.getPower());
        telemetry.addData("power FL", leftFront.getPower());
        telemetry.update();
    }

    public void trackPosition() throws InterruptedException {
        pause(.05);
        //trig calculated before changeInParallelPos determined because the calculation takes too long and otherwise skews distance
        double cosAngle = Math.cos(currentAngle()*Math.PI/180);
        double sinAngle = Math.sin(currentAngle()*Math.PI/180);
        double changeInParallelPos = (parallelEncoderTracker.getCurrentPosition() - initialParallelEncoderPosition)*PARALLEL_INCHES_OVER_TICKS;
        //int changeInPerpendicularPos = perpendicularEncoderTracker.getCurrentPosition() - initialPerpendicularEncoderPosition;

        //sign of perpendicularPos trig may be subject to change in future when we test
        xPos += (changeInParallelPos*cosAngle); //* Math.cos(currentAngle()); //- changeInPerpendicularPos * Math.sin(currentAngle()));
        yPos += (changeInParallelPos*sinAngle); //+ changeInPerpendicularPos * Math.cos(currentAngle()));

        initialParallelEncoderPosition = parallelEncoderTracker.getCurrentPosition();
        initialPerpendicularEncoderPosition = perpendicularEncoderTracker.getCurrentPosition();

        telemetry.addData("change in parallelPos", changeInParallelPos);
        telemetry.addData("Cos angle", Math.cos(currentAngle()*Math.PI/180));
        telemetry.addData("x pos", xPos);
        telemetry.addData("y pos", yPos);
        telemetry.update();
    }

    public void halt() {
        for (DcMotorEx motor : motors) {
            motor.setPower(0);
        }
    }

    public void pause(double time) throws InterruptedException {
        double pause = runtime.time();
        //pause robot for "pause" seconds
        while (runtime.time() - pause < time) {
            telemetry.addData("XPos", xPos);
            telemetry.addData("YPos", yPos);
            telemetry.update();
            heartbeat();
        }
    }

    public void adjustClaw() {
        leftClaw.setPosition(.6);
        rightClaw.setPosition(.5);
    }

    public void gripBlock() {
        leftClaw.setPosition(0);
        rightClaw.setPosition(1);
    }

    private void heartbeat() throws InterruptedException{
        //if opMode is stopped, will throw and catch an InterruptedException rather than resulting in red text and program crash on phone
        if(!opModeIsActive()) {
            throw new InterruptedException();
        }
    }
}
