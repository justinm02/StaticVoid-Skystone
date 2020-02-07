package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.teamcode.MotionProfiling.MotionProfiler;
import org.firstinspires.ftc.teamcode.Autonomous.OpenCV.OpenCV;
import org.firstinspires.ftc.teamcode.Autonomous.PID.PID;
import org.firstinspires.ftc.teamcode.Autonomous.Spline.Bezier;
import org.firstinspires.ftc.teamcode.Autonomous.Spline.Waypoint;
import org.firstinspires.ftc.teamcode.Autonomous.Vuforia.Vuforia;
import org.firstinspires.ftc.teamcode.Localization.PositionTracker;

import com.qualcomm.robotcore.util.Range;

public abstract class Auto extends LinearOpMode {
    public ElapsedTime runtime = new ElapsedTime();
    private BNO055IMU imu;
    private DcMotorEx leftFront, leftBack, rightFront, rightBack, rightIntake, leftIntake, leftVerticalSlide, rightVerticalSlide;
    public Servo blockClaw, blockRotator, autoBlockGrabber, blockAligner;
    private CRServo horizontalSlide;
    private TouchSensor horizontalLimit, lowerVerticalLimit;
    private PositionTracker positionTracker = new PositionTracker(0, 0, 0);
    private DcMotorEx[] motors = {leftFront, leftBack, rightFront, rightBack};
    //private PID forwardHeadingPID = new PID(0.04, 0, 0.0024);
    private PID forwardHeadingPID = new PID(0, 0, 0);
    private Servo leftPlatformLatcher, rightPlatformLatcher;
    private PID strafePID = new PID(0.04, 0, 0.0038);
    //private PID strafePID = new PID(0, 0, 0);
    private PID xPositionPID = new PID(0.09, 0, 0);
    //private PID xPositionPID = new PID(0, 0, 0);
    private PID yPositionPID = new PID(0, 0, 0);
    public int baseParallelLeftPosition, basePerpendicularPosition, baseParallelRightPosition;
    private double xPos = 0;
    private double yPos = 0;
    private final double GEAR_RATIO = 1.00000, WHEEL_DIAMETER = 4, WHEEL_TICKS_PER_REV = 560;
    // C = circumference
    private final double C = WHEEL_TICKS_PER_REV / (Math.PI * WHEEL_DIAMETER * GEAR_RATIO), STRAFE_COEFFICIENT = 1.20;
    private final int deadWheelTicks = 4096;
    private final double WHEEL_CIRCUMFERENCE_IN = Math.PI * 3.05; //circumference of parallel deadwheel
    private final double DEADWHEEL_INCHES_OVER_TICKS = WHEEL_CIRCUMFERENCE_IN / deadWheelTicks; //conversion used to convert inches to ticks (what encoderTracker.getCurrentPosition() reads)

    private Vuforia vuforia = new Vuforia();
    private OpenCV openCV = new OpenCV();

    private MotionProfiler motionProfiler;

    private int cameraMonitorViewId;
    private VuforiaLocalizer.Parameters parameters;

    public enum SKYSTONE_POSITION {
        LEFT,
        MIDDLE,
        RIGHT
    }

    public void initialize() {
        initMotors();
        initServos();
        initGyro();

        openCV.initCamera(hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName()));

        runtime = new ElapsedTime();

        xPos = 0;
        yPos = 0;

        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    public void initMotors() {
        leftFront = (DcMotorEx) hardwareMap.dcMotor.get("leftFront");
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack = (DcMotorEx) hardwareMap.dcMotor.get("leftBack");
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront = (DcMotorEx) hardwareMap.dcMotor.get("rightFront");
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack = (DcMotorEx) hardwareMap.dcMotor.get("rightBack");
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //Right and left motors facing opposite direction so right motors set to reverse
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.REVERSE);

        motors = new DcMotorEx[]{leftFront, leftBack, rightFront, rightBack};

        for (DcMotorEx motor : motors) {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        resetMotors();

        leftIntake = (DcMotorEx) hardwareMap.dcMotor.get("leftIntake");
        leftIntake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftIntake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftIntake.setDirection(DcMotor.Direction.REVERSE);


        rightIntake = (DcMotorEx) hardwareMap.dcMotor.get("rightIntake");
        rightIntake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightIntake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightIntake.setDirection(DcMotor.Direction.REVERSE);

        leftVerticalSlide = (DcMotorEx) hardwareMap.dcMotor.get("leftVerticalSlide");
        leftVerticalSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftVerticalSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rightVerticalSlide = (DcMotorEx) hardwareMap.dcMotor.get("rightVerticalSlide");
        rightVerticalSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightVerticalSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightVerticalSlide.setDirection(DcMotor.Direction.REVERSE);

        lowerVerticalLimit = hardwareMap.touchSensor.get("lowerVerticalLimit");

        baseParallelLeftPosition = leftIntake.getCurrentPosition();
        baseParallelRightPosition = rightVerticalSlide.getCurrentPosition();
        basePerpendicularPosition = rightIntake.getCurrentPosition();
    }

    public void initServos() {
        horizontalSlide = hardwareMap.get(CRServo.class, "horizontalSlide");
        horizontalSlide.setDirection(DcMotorSimple.Direction.REVERSE);

        horizontalLimit = hardwareMap.touchSensor.get("horizontalLimit");

        blockClaw = hardwareMap.servo.get("blockClaw");
        blockRotator = hardwareMap.servo.get("blockRotator");
        leftPlatformLatcher = hardwareMap.servo.get("leftPlatformLatcher");
        rightPlatformLatcher = hardwareMap.servo.get("rightPlatformLatcher");
        autoBlockGrabber = hardwareMap.servo.get("autoBlockGrabber");
        blockAligner = hardwareMap.servo.get("blockAligner");

        blockClaw.setPosition(.26);
        blockRotator.setPosition(.485);

        leftPlatformLatcher.setPosition(0);
        rightPlatformLatcher.setPosition(0);
        autoBlockGrabber.setPosition(0);
        blockAligner.setPosition(.2);

        //test
        leftPlatformLatcher.setPosition(0);
        rightPlatformLatcher.setPosition(0);
        autoBlockGrabber.setPosition(0);
        blockAligner.setPosition(.2);
    }

    public void initGyro() {
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imu.initialize(parameters);
    }

    public SKYSTONE_POSITION determineSkystonePosition(String team) throws InterruptedException {
        openCV.openCamera(team);
        pause(0.5);

        int[] detectionVals = openCV.detectSkystone();

        if (detectionVals[0] == 0) {
            return SKYSTONE_POSITION.LEFT;
        }
        else if (detectionVals[1] == 0) {
            return SKYSTONE_POSITION.MIDDLE;
        }
        else {
            if (detectionVals[2] != 0) {
                telemetry.addData("POSITION FAILED", SKYSTONE_POSITION.RIGHT);
                telemetry.update();
            }
            return SKYSTONE_POSITION.RIGHT;
        }
    }

    public double getError(double current, double target) {
        double error;

        double error1 = /*current - target*/ target - current;
        double error2;
        if (current < 0)
            error2 = /*(current + 360) - (target);*/ target - (current + 360);
        else
            error2 = /*(current - 360) - (target);*/ target - (current - 360);
        if (Math.abs(error1) <= Math.abs(error2))
            error = error1;
        else
            error = error2;

        return error;
    }

    public void move(double targetHeading, double magnitude, int direction, double maximumPower, double initPower, double finalPower, String movement, boolean halt) throws InterruptedException {
        double baseParallelLeftTicks = leftIntake.getCurrentPosition();
        double baseParallelRightTicks = rightVerticalSlide.getCurrentPosition();
        double basePerpendicularTicks = rightIntake.getCurrentPosition();
        double directionRadians = Math.toRadians(direction - targetHeading);

        double baseXPosition = positionTracker.getCurrentX();
        double baseYPosition = positionTracker.getCurrentY();

        double sRight = 0;
        double sLeft = 0;
        double sAvg = 0;
        double sStrafe = 0;

        double headingError;
        double headingCorrection;

        double xPositionError;
        double xPositionCorrection;

        double yPositionError;
        double yPositionCorrection;

        double dTravelled = 0;
        double parallelLeftTicks = 0;
        double parallelRightTicks = 0;
        double perpendicularTicks = 0;

        double currentPower = 0;

        PID headingPID;

        if (movement.contains("strafe")) {
            headingPID = strafePID;
            motionProfiler = new MotionProfiler(.125);
        } else {
            headingPID = forwardHeadingPID;
            motionProfiler = new MotionProfiler(.125);
        }

        while (dTravelled < magnitude) {
            double targetX = Math.cos(directionRadians)*dTravelled;
            double targetY = Math.sin(directionRadians)*dTravelled;

            //when axis between -179 and 179 degrees is crossed, degrees must be converted from 0 - 360 degrees. 179-(-179) = 358. 179 - 181 = -2. Big difference
            headingError = getError(currentAngle(), targetHeading);

            double errorX = targetX - ((positionTracker.getCurrentX() - baseXPosition));
            double errorY = targetY - ((positionTracker.getCurrentY() - baseYPosition));

//            telemetry.addData("error", error);
//            telemetry.update();

            headingCorrection = headingPID.getCorrection(headingError, runtime);
            xPositionCorrection = xPositionPID.getCorrection(errorX, runtime);
            yPositionCorrection = yPositionPID.getCorrection(errorY, runtime);

            double proportionTravelled = dTravelled / magnitude;

            currentPower = motionProfiler.getProfilePower(proportionTravelled, maximumPower, initPower, finalPower);

            double leftFrontPower = Math.sin(directionRadians + 3 * Math.PI / 4) - headingCorrection + xPositionCorrection - yPositionCorrection;
            double leftBackPower = Math.sin(directionRadians + Math.PI / 4) - headingCorrection + xPositionCorrection + yPositionCorrection;
            double rightFrontPower = Math.sin(directionRadians + Math.PI / 4) + headingCorrection + xPositionCorrection + yPositionCorrection;
            double rightBackPower = Math.sin(directionRadians + 3 * Math.PI / 4) + headingCorrection + xPositionCorrection - yPositionCorrection;

            double conversion = Math.abs(currentPower / getMaxMagnitude(new double[]{leftFrontPower, leftBackPower, rightFrontPower, rightBackPower}));

            leftFront.setPower(conversion * leftFrontPower);
            leftBack.setPower(conversion * leftBackPower);
            rightFront.setPower(conversion * rightFrontPower);
            rightBack.setPower(conversion * rightBackPower);

            parallelLeftTicks = leftIntake.getCurrentPosition() - baseParallelLeftTicks;
            parallelRightTicks = rightVerticalSlide.getCurrentPosition() - baseParallelRightTicks;
            perpendicularTicks = rightIntake.getCurrentPosition() - basePerpendicularTicks;

            sRight = (parallelRightTicks) * DEADWHEEL_INCHES_OVER_TICKS;
            sLeft = (parallelLeftTicks) * DEADWHEEL_INCHES_OVER_TICKS;
            sAvg = (sLeft + sRight) / 2;
            sStrafe = perpendicularTicks * DEADWHEEL_INCHES_OVER_TICKS;

            dTravelled = Math.sqrt(Math.pow(sAvg, 2) + Math.pow(sStrafe, 2));
            telemetry.addData("dTravelled", dTravelled);
            telemetry.addData("sLeft", sLeft);
            telemetry.addData("sRight", sRight);
            telemetry.update();

            positionTracker.updateTicks(parallelLeftTicks, parallelRightTicks, perpendicularTicks);
            positionTracker.updateLocationAndPose(telemetry, currentAngle(), "move");

            /*telemetry.addData("dTravelled", dTravelled);
            telemetry.addData("sLeft", sLeft);
            telemetry.addData("sRight", sRight);
            telemetry.addData("sAvg", sAvg);
            telemetry.addData("Right ticks", parallelRightTicks);
            telemetry.addData("Left ticks", parallelLeftTicks);
            telemetry.addData("Strafe ticks", perpendicularTicks);
            telemetry.update();*/

            heartbeat();

            /*telemetry.addData("X value", positionTracker.getCurrentX());
            telemetry.addData("Y value", positionTracker.getCurrentY());
            telemetry.addData("currentAngle", currentAngle());
            telemetry.addData("deadwheel angle", positionTracker.getCurrentAngle());*/
            telemetry.addData("actual left front power", leftFront.getPower());
            telemetry.addData("actual right front power", rightFront.getPower());
            telemetry.addData("actual left back power", leftBack.getPower());
            telemetry.addData("actual right back power", rightBack.getPower());
            telemetry.addData("errorX", errorX);
            telemetry.addData("targetXPosition", targetX);
            telemetry.addData("Current X", positionTracker.getCurrentX() - baseXPosition);
            telemetry.addData("Current X", positionTracker.getCurrentY() - baseYPosition);
            telemetry.update();
        }

        if (halt) {
            halt();
        }
    }

    public void moveByWheelEncoders(double targetHeading, double inches, double power, String movementType) throws InterruptedException {
        resetMotors();

        double currentPosition = leftBack.getCurrentPosition();

        int ticks = (int)(inches * C);

        while (motorsBusy(ticks, currentPosition)) {
            heartbeat();
            correction(power, targetHeading, movementType, false, 1);
        }

        halt();
    }

    public void strafeByWheelEncoders(double targetHeading, double inches, double power, String movementType) throws InterruptedException {
        resetMotors();

        double currentPosition = leftBack.getCurrentPosition();

        int ticks = (int)(inches * C * STRAFE_COEFFICIENT);

        while (motorsBusy(ticks, currentPosition)) {
            heartbeat();
            correction(power, targetHeading, movementType, false, 1);
        }

        halt();
    }

    public void splineMove(double[] xcoords, double[] ycoords, double maximumPower, double initPower, double finalPower, double offset, boolean halt) throws InterruptedException {
        double baseParallelLeftTicks = leftIntake.getCurrentPosition();
        double baseParallelRightTicks = rightVerticalSlide.getCurrentPosition();
        double basePerpendicularTicks = rightIntake.getCurrentPosition();

        double parallelLeftTicks = 0;
        double parallelRightTicks = 0;
        double perpendicularTicks = 0;

        double sRight = 0;
        double sLeft = 0;
        double sAvg = 0;

        //double startingPosition = parallelEncoderTracker.getCurrentPosition();
        Waypoint[] coords = new Waypoint[xcoords.length];
        boolean inverted = false;

        //set Waypoints per each (x,y)
        for (int i = 0; i < coords.length; i++) {
            coords[i] = new Waypoint(xcoords[i], ycoords[i]);
        }

        //if spline backwards, set inverted to true (lets correction method know to make adjustments to targetHeading in PD correction method)
        if (maximumPower < 0) {
            inverted = true;
        }

        //sets new spline, defines important characteristics
        Bezier spline = new Bezier(coords);
        double t = 0;
        double distanceTraveled;
        double arclength = spline.getArcLength(); //computes arc length by adding infinitesimally small slices of sqrt( (dx/dt)^2 + (dy/dt)^2 ) (distance formula). This method uses integration, a fundamental component in calculus

        motionProfiler = new MotionProfiler(.125);
        double currentPower = 0;

        while (t <= 1.0) {
            heartbeat();

            currentPower = motionProfiler.getProfilePower(t, maximumPower, initPower, finalPower);
            //constantly adjusts heading based on what the current spline angle should be based on the calculated t
            correction(currentPower, (int) (Math.toDegrees(spline.getAngle(t, offset))), "spline", inverted, 1.0);
            //distanceTraveled computed by converting encoderTraveled ticks on deadwheel to inches traveled
            parallelLeftTicks = leftIntake.getCurrentPosition() - baseParallelLeftTicks;
            parallelRightTicks = rightVerticalSlide.getCurrentPosition() - baseParallelRightTicks;
            perpendicularTicks = rightIntake.getCurrentPosition() - basePerpendicularTicks;

            sRight = (parallelRightTicks) * DEADWHEEL_INCHES_OVER_TICKS;
            sLeft = (parallelLeftTicks) * DEADWHEEL_INCHES_OVER_TICKS;
            sAvg = (sLeft + sRight) / 2;

            distanceTraveled = sAvg;

            positionTracker.updateTicks(parallelLeftTicks, parallelRightTicks, perpendicularTicks);
            positionTracker.updateLocationAndPose(telemetry, currentAngle(), "spline");

            t = Math.abs(distanceTraveled / arclength);
        }
        if (halt) {
            halt();
        }
    }

    public double getMaxMagnitude(double[] arr) {
        double max = Math.abs(arr[0]);

        for (int i = 1; i < arr.length; i++) {
            if (Math.abs(arr[i]) > max) {
                max = arr[i];
            }
        }

        return max;
    }

    public void moveByTime(double time, double power, double targetHeading) throws InterruptedException {
        double current = runtime.time();

        while (runtime.time() - current < time) {
            heartbeat();
            double target = targetHeading;
            double currentAngle = currentAngle();

            //when axis between -179 and 179 degrees is crossed, degrees must be converted from 0 - 360 degrees. 179-(-179) = 358. 179 - 181 = -2. Big difference
            double error = getError(currentAngle, target);

            double correction = forwardHeadingPID.getCorrection(error, runtime);

            leftFront.setPower(power - correction);
            leftBack.setPower(power - correction);
            rightFront.setPower(power + correction);
            rightBack.setPower(power + correction);
        }
        halt();
    }

    public void slideAndMoveByTime(double time, double power, double targetHeading) throws InterruptedException {
        double current = runtime.time();

        while (runtime.time() - current < 0.5) {
            heartbeat();
            leftIntake.setPower(1);
            rightIntake.setPower(1);
        }
        blockClaw.setPosition(1);
        leftIntake.setPower(0);
        rightIntake.setPower(0);


        while (runtime.time() - current < time + 0.5) {
            heartbeat();
            double target = targetHeading;
            double currentAngle = currentAngle();

            //when axis between -179 and 179 degrees is crossed, degrees must be converted from 0 - 360 degrees. 179-(-179) = 358. 179 - 181 = -2. Big difference
            double error = getError(currentAngle, target);

            double correction = forwardHeadingPID.getCorrection(error, runtime);

            leftFront.setPower(power - correction);
            leftBack.setPower(power - correction);
            rightFront.setPower(power + correction);
            rightBack.setPower(power + correction);

            horizontalSlide.setPower(.6);

            telemetry.addData("error", error);
            telemetry.update();
        }
        horizontalSlide.setPower(0);

        leftIntake.setPower(0);
        rightIntake.setPower(0);

        blockClaw.setPosition(0.1);

        halt();
    }

    public void PIDTurn(int targetHeading, double max) throws InterruptedException {
        double error = getError(currentAngle(), targetHeading);
        while (Math.abs(error) > 3.5) {
            error = getError(currentAngle(), targetHeading);

            telemetry.addData("error", error);
            telemetry.update();

            double correction = forwardHeadingPID.getCorrection(error, runtime);

            leftBack.setPower(Range.clip(-correction, -max, max));
            leftFront.setPower(Range.clip(-correction, -max, max));
            rightBack.setPower(Range.clip(correction, -max, max));
            rightFront.setPower(Range.clip(correction, -max, max));

            heartbeat();
        }

        halt();
    }

    public double currentAngle() {
        //returns heading from gyro using unit circle values (-180 to 180 degrees, -pi to pi radians. We're using degrees)
        return imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
    }

    public void correction(double power, double targetHeading, String movementType, boolean inverted, double max) {
        //sets target and current angles
        double target = targetHeading;
        double current = currentAngle();

        //if the spline motion is backwards, the target must be flipped 180 degrees in order to match with spline.getAngle()
        if (inverted && movementType.contains("spline")) {
            target = (targetHeading > 0) ? (targetHeading - 180) : (targetHeading + 180);
        }

        //when axis between -179 and 179 degrees is crossed, degrees must be converted from 0 - 360 degrees. 179-(-179) = 358. 179 - 181 = -2. Big difference
        double error = getError(current, target);

        //PD correction for both regular and spline motion
        if (movementType.contains("straight") || movementType.contains("spline")) {
            double correction = forwardHeadingPID.getCorrection(error, runtime);

            double leftPower = Range.clip(power - correction, -max, max);
            double rightPower = Range.clip(power + correction, -max, max);

            leftFront.setPower(leftPower);
            rightFront.setPower(rightPower);
            leftBack.setPower(leftPower);
            rightBack.setPower(rightPower);
            telemetry.addData("left expected power", leftPower);
            telemetry.addData("right expected power", rightPower);
            telemetry.addData("actual left power", leftFront.getPower());
            telemetry.addData("actual right power", rightFront.getPower());
        }

        //pd correction for strafe motion. Right and left are opposites
        else if (movementType.contains("strafe")) {
            double correction = strafePID.getCorrection(error, runtime);

            if (movementType.contains("left")) {
                leftFront.setPower(Range.clip(-power - correction, -1.0, 1.0));
                rightFront.setPower(Range.clip(power + correction, -1.0, 1.0));
                leftBack.setPower(Range.clip(power - correction, -1.0, 1.0));
                rightBack.setPower(Range.clip(-power + correction, -1.0, 1.0));
            } else if (movementType.contains("right")) {
                leftFront.setPower(Range.clip(power - correction, -1.0, 1.0));
                rightFront.setPower(Range.clip(-power + correction, -1.0, 1.0));
                leftBack.setPower(Range.clip(-power - correction, -1.0, 1.0));
                rightBack.setPower(Range.clip(power + correction, -1.0, 1.0));
            }
        }

        telemetry.addData("current Angle", current);
        telemetry.addData("target", target);
        telemetry.addData("error", error);
        telemetry.addData("lf", leftFront.getPower());
        telemetry.addData("lb", leftBack.getPower());
        telemetry.addData("rf", rightFront.getPower());
        telemetry.addData("rb", rightBack.getPower());
        telemetry.addData("avg power", (rightBack.getPower() + rightFront.getPower() + leftBack.getPower() + leftFront.getPower()) / 4);
        telemetry.update();
    }

    public void halt() {
        for (DcMotorEx motor : motors) {
            motor.setPower(0);
        }
    }

    public boolean motorsBusy(int ticks, double startingPosition) {
        return Math.abs(leftBack.getCurrentPosition() - startingPosition) < ticks && Math.abs(rightBack.getCurrentPosition() - startingPosition) < ticks && Math.abs(leftFront.getCurrentPosition() - startingPosition) < ticks && Math.abs(rightFront.getCurrentPosition() - startingPosition) < ticks;
    }

    public void resetMotors() {
        for (DcMotorEx motor : motors) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        }
    }

    public void intake() {
        leftIntake.setPower(.65);
        rightIntake.setPower(.65);
        horizontalSlide.setPower(-.4);
    }

    public void stopIntake() {
        leftIntake.setPower(0);
        rightIntake.setPower(0);
        horizontalSlide.setPower(0);
    }

    public void gripPlatform() {
        leftPlatformLatcher.setPosition(1);
        rightPlatformLatcher.setPosition(1);

        //test
        rightPlatformLatcher.setPosition(1);
        leftPlatformLatcher.setPosition(1);
    }

    public void releasePlatform() {
        leftPlatformLatcher.setPosition(0);
        rightPlatformLatcher.setPosition(0);
    }

    public void bringAlignerDown() {
        blockAligner.setPosition(.625);
    }

    public void grip() {
        autoBlockGrabber.setPosition(1);
    }

    public void gripBlock() throws InterruptedException {
        blockAligner.setPosition(1);
        pause(.25);
        autoBlockGrabber.setPosition(1);
    }

    public void bringAlignerUp() {
        blockAligner.setPosition(.2);
    }

    public void releaseBlock() {
        autoBlockGrabber.setPosition(0);
    }

    public void extendHorizontalSlide() {
        double currentTime = runtime.time();

        while (runtime.time() - currentTime < 2.5) {
            horizontalSlide.setPower(-1);
        }
        horizontalSlide.setPower(0);
    }

    public void retractHorizontalSlide() {
        if (horizontalLimit.getValue() < 1) {
            horizontalSlide.setPower(1);
        }
        horizontalSlide.setPower(0);
    }

    public void extendVerticalLift() {
        double currentTime = runtime.time();

        while (runtime.time() - currentTime < 0.5) {
            rightVerticalSlide.setPower(1);
        }
        rightVerticalSlide.setPower(0);
    }

    public void retractVerticalLift() {
        while (lowerVerticalLimit.getValue() < 1) {
            rightVerticalSlide.setPower(-.7);
        }
        rightVerticalSlide.setPower(0);
    }

    public void pause(double time) throws InterruptedException {
        double pause = runtime.time();
        //pause robot for "pause" seconds
        while (runtime.time() - pause < time) {
            heartbeat();
            /*telemetry.addData("current x: ", positionTracker.getCurrentX());
            telemetry.addData("current y: ", positionTracker.getCurrentY());
            telemetry.update();*/
        }
    }

    public void heartbeat() throws InterruptedException {
        //if opMode is stopped, will throw and catch an InterruptedException rather than resulting in red text and program crash on phone
        if (!opModeIsActive()) {
            throw new InterruptedException();
        }
    }
}