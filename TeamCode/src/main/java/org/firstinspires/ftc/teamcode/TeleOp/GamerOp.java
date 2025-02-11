package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Localization.PositionTracker;

public class GamerOp extends OpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotorEx leftFront, leftBack, rightFront, rightBack, rightIntake, leftIntake, leftVerticalSlide, rightVerticalSlide;
    private Servo blockClaw, backBlockClaw, leftFlipper, rightFlipper, leftPlatformLatcher, rightPlatformLatcher, rightAutoBlockGrabber, rightBlockAligner, leftAutoBlockGrabber, leftBlockAligner, capstoneDeployer, staffServo;
    private CRServo horizontalSlide;
    private TouchSensor horizontalLimit, lowerVerticalLimit;
    private boolean precision, direction, precisionChanged, directionChanged;
    private boolean useOneGamepad;
    private boolean positionChanged = false;
    private boolean closed = false;
    private boolean deployerPositionChanged = false;
    private boolean deployerClosed = false;
    private double baseParallelRightPosition;
    private double baseParallelLeftPosition;
    private double basePerpendicularPosition;
    private final int deadWheelTicks = 4096;
    private final double WHEEL_CIRCUMFERENCE_IN = Math.PI*3.05; //circumference of parallel deadwheel
    private final double DEADWHEEL_INCHES_OVER_TICKS = WHEEL_CIRCUMFERENCE_IN/deadWheelTicks;
    private double time = -999;
    private PositionTracker positionTracker = new PositionTracker(0, 0, 0);

    @Override
    public void init() {
        //after driver hits init
        setUpDriveTrain();
        setUpIntake();
        setUpSlides();
        setUpServos();

        precision = false;
        direction = false;
        useOneGamepad = false;
        precisionChanged = false;
        directionChanged = false;

        closed = false;
        positionChanged = false;

        telemetry.addData("Status", "Initialized");

        baseParallelLeftPosition = leftIntake.getCurrentPosition();
        baseParallelRightPosition = rightVerticalSlide.getCurrentPosition();
        basePerpendicularPosition = rightIntake.getCurrentPosition();
    }

    public void setUpDriveTrain() {
        leftFront = (DcMotorEx) hardwareMap.dcMotor.get("leftFront");
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBack = (DcMotorEx) hardwareMap.dcMotor.get("leftBack");
        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront = (DcMotorEx) hardwareMap.dcMotor.get("rightFront");
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack = (DcMotorEx) hardwareMap.dcMotor.get("rightBack");
        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //Right and left motors facing opposite direction so right motors set to reverse
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.REVERSE);
    }

    public void setUpIntake() {
        leftIntake = (DcMotorEx) hardwareMap.dcMotor.get("leftIntake");
        leftIntake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //leftIntake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //leftIntake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftIntake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        rightIntake = (DcMotorEx) hardwareMap.dcMotor.get("rightIntake");
        rightIntake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightIntake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightIntake.setDirection(DcMotor.Direction.REVERSE);
    }

    public void setUpSlides() {
        leftVerticalSlide = (DcMotorEx) hardwareMap.dcMotor.get("leftVerticalSlide");
        leftVerticalSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftVerticalSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //verticalSlide.setDirection(DcMotor.Direction.REVERSE);

        rightVerticalSlide = (DcMotorEx) hardwareMap.dcMotor.get("rightVerticalSlide");
        rightVerticalSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightVerticalSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightVerticalSlide.setDirection(DcMotor.Direction.REVERSE);

        horizontalSlide = hardwareMap.get(CRServo.class, "horizontalSlide");
        //horizontalSlide.setDirection(DcMotorSimple.Direction.REVERSE);

        horizontalLimit = hardwareMap.touchSensor.get("horizontalLimit");
        lowerVerticalLimit = hardwareMap.touchSensor.get("lowerVerticalLimit");
    }

    public void setUpServos() {
        blockClaw = hardwareMap.servo.get("blockClaw");
        backBlockClaw = hardwareMap.servo.get("backBlockClaw");

        leftFlipper = hardwareMap.servo.get("leftFlipper");
        rightFlipper = hardwareMap.servo.get("rightFlipper");

        leftPlatformLatcher = hardwareMap.servo.get("leftPlatformLatcher");
        rightPlatformLatcher = hardwareMap.servo.get("rightPlatformLatcher");
        rightPlatformLatcher.setDirection(Servo.Direction.REVERSE);

        rightAutoBlockGrabber = hardwareMap.servo.get("rightAutoBlockGrabber");
        rightBlockAligner = hardwareMap.servo.get("rightBlockAligner");
        //rightAutoBlockGrabber.setDirection(Servo.Direction.REVERSE);

        //leftAutoBlockGrabber = hardwareMap.servo.get("leftAutoBlockGrabber");
        //leftBlockAligner = hardwareMap.servo.get("leftBlockAligner");

        capstoneDeployer = hardwareMap.servo.get("capstoneDeployer");

        //staffServo = hardwareMap.servo.get("staffServo");

        blockClaw.setPosition(.1);
        leftPlatformLatcher.setPosition(0);
        rightPlatformLatcher.setPosition(0);
        rightAutoBlockGrabber.setPosition(1);
        rightBlockAligner.setPosition(.2);
        //staffServo.setPosition(1);

        //test
//        blockClaw.setPosition(.1);
//        leftPlatformLatcher.setPosition(0);
//        rightPlatformLatcher.setPosition(0);
        //leftAutoBlockGrabber.setPosition(1);
        //leftBlockAligner.setPosition(1);
    }

    @Override
    //what runs in between robot being initialized and before it plays
    public void init_loop() {

    }

    //once driver hits play
    @Override
    public void start() {
        runtime.reset();
    }


    @Override
    public void loop() {
        telemetry.update();

        //useEncoders();
        driveBot();
        intake();
        moveSlides();
        gripBlock();
        gripPlatform();
        autoGripBlock();
        deployCapstone();
        useEncoders();
        //launchStaff();

        bringDownLiftAutomated();

        useOneGamepad();
    }

    public void useEncoders() {
        double parallelLeftTicks = (leftIntake.getCurrentPosition() - baseParallelLeftPosition);
        double parallelRightTicks = rightVerticalSlide.getCurrentPosition() - baseParallelRightPosition;
        double perpendicularTicks = rightIntake.getCurrentPosition() - basePerpendicularPosition;

        positionTracker.updateTicks(parallelLeftTicks, parallelRightTicks, perpendicularTicks);

        if(Math.abs(gamepad1.left_stick_x) < Math.cos(Math.toRadians(5)) || Math.sqrt(Math.pow(gamepad1.left_stick_x,2) + Math.pow(gamepad1.left_stick_y,2)) < .1 ) {
            positionTracker.updateLocationAndPose(telemetry, "");
        }
        else {
            positionTracker.updateLocationAndPose(telemetry, "");
        }

        telemetry.addData("parallel left ticks", parallelLeftTicks);
        telemetry.addData("parallel right ticks", parallelRightTicks);
        telemetry.addData("perpendicular ticks", perpendicularTicks);
        telemetry.addData("Distance traveled", (parallelLeftTicks + parallelRightTicks)*DEADWHEEL_INCHES_OVER_TICKS/2);
        telemetry.addData("X Position", positionTracker.getCurrentX());
        telemetry.addData("Y Position", positionTracker.getCurrentY());
        telemetry.update();
    }

    public void driveBot() {
        if ((gamepad1.a && !precisionChanged && !useOneGamepad) || (useOneGamepad && gamepad1.y)) {
            precision = !precision;
            precisionChanged = true;
        }
        else if ((!useOneGamepad && !gamepad1.a) || (useOneGamepad && !gamepad1.y)) {
            precisionChanged = false;
        }

//        if (gamepad1.x && !directionChanged && !useOneGamepad) {
//            direction = !direction;
//            directionChanged = true;
//        }
//        else if (!gamepad1.x && !useOneGamepad) {
//            directionChanged = false;
//        }

        double xMagnitude = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y);
        double xLinear = direction ? xMagnitude : -xMagnitude;

        double joystickAngle = Math.atan2(gamepad1.left_stick_y, gamepad1.left_stick_x) + 3*Math.PI/4;
        double xTurn = gamepad1.right_stick_x;


        double leftFrontPower = xLinear*Math.sin(joystickAngle) - xTurn;
        double rightFrontPower = xLinear*Math.cos(joystickAngle) + xTurn;
        double leftBackPower = xLinear*Math.cos(joystickAngle) - xTurn;
        double rightBackPower = xLinear*Math.sin(joystickAngle) + xTurn;

        double[] motorPowers = new double[]{leftFrontPower, rightFrontPower, leftBackPower, rightBackPower};
        motorPowers = convertMotorPowers(motorPowers, xLinear, xTurn);

        leftFront.setPower(precision ? 0.4 * motorPowers[0] : motorPowers[0]);
        rightFront.setPower(precision ? 0.4 * motorPowers[1] : motorPowers[1]);
        leftBack.setPower(precision ? 0.4 * motorPowers[2] : motorPowers[2]);
        rightBack.setPower(precision ? 0.4 * motorPowers[3] : motorPowers[3]);

//        telemetry.addData("Front Motors", "Left Front (%.2f), Right Front (%.2f)", leftFront.getPower(), rightFront.getPower());
//        telemetry.addData("joystickAngle", joystickAngle);
//        telemetry.addData("Rear Motors", "Left Rear (%.2f), Right Rear (%.2f)", leftBack.getPower(), rightBack.getPower());
    }

    public double[] convertMotorPowers(double[] motorPowers, double xLinear, double xTurn) {
        double maxPower = getMaxMagnitude(motorPowers);

        double conversion = Math.abs(Math.sqrt((Math.pow(xLinear,2) + Math.pow(xTurn, 2))/*/2*/)/maxPower);

//        telemetry.addData("maxPower", maxPower);
//        telemetry.addData("conversion", conversion);
//        telemetry.update();

        for (int i = 0; i < motorPowers.length; i++) {
            motorPowers[i] *= conversion;
        }

        return motorPowers;
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

    public void intake() {
        if (gamepad1.right_bumper) {
            leftIntake.setPower(1);
            rightIntake.setPower(1);
        }
        else if (gamepad1.left_bumper) {
            leftIntake.setPower(-1);
            rightIntake.setPower(-1);
        }
        else {
            leftIntake.setPower(0);
            rightIntake.setPower(0);
        }
    }

    public void moveSlides() {
        //vertical slide
        if((lowerVerticalLimit.getValue() < 1 || gamepad2.left_stick_y < 0 && !useOneGamepad)) {
            rightVerticalSlide.setPower(-gamepad2.left_stick_y*(gamepad2.left_stick_y > 0 ? .5 : 1));
            leftVerticalSlide.setPower(-gamepad2.left_stick_y*(gamepad2.left_stick_y > 0 ? .5 : 1));
        }
        else {
            if (!useOneGamepad) {
                rightVerticalSlide.setPower(0);
                leftVerticalSlide.setPower(0);
            }
        }

        if (useOneGamepad && gamepad1.dpad_up) {
            rightVerticalSlide.setPower(1);
            leftVerticalSlide.setPower(1);
        }
        else if (lowerVerticalLimit.getValue() < 1 && useOneGamepad && gamepad1.dpad_down) {
            rightVerticalSlide.setPower(-1);
            leftVerticalSlide.setPower(-1);
        }
        else {
            if (useOneGamepad) {
                rightVerticalSlide.setPower(0);
                leftVerticalSlide.setPower(0);
            }
        }

        if (((gamepad2.left_stick_y == 0 && !useOneGamepad) || (useOneGamepad && !gamepad1.dpad_up && !gamepad1.dpad_down)) && lowerVerticalLimit.getValue() < 1) {
            rightVerticalSlide.setPower(.05);
            leftVerticalSlide.setPower(.05);
        }

        //horizontal slide
        if (!useOneGamepad) {
            horizontalSlide.setPower(Range.clip(-gamepad2.right_stick_y, -.85, .85));
        }
        else {
            if (gamepad1.dpad_left) {
                horizontalSlide.setPower(-.85);
            }
            else if (gamepad1.dpad_right) {
                horizontalSlide.setPower(.85);
            }
            else {
                horizontalSlide.setPower(0);
            }
        }

        /*telemetry.addData("slide power", verticalSlide.getPower());
        telemetry.addData("slide position", verticalSlide.getCurrentPosition());
        telemetry.addData("horizontal limit", horizontalLimit.getValue());
        telemetry.update();*/
    }

    public void gripBlock() {
        if ((gamepad2.b || (useOneGamepad && gamepad1.b)) && !gamepad2.start && !positionChanged) {
            blockClaw.setPosition(closed ? 1 : 0.1);
            closed = !closed;
            positionChanged = true;
        }
        else if ((!useOneGamepad && !gamepad2.b) || (useOneGamepad && !gamepad1.b)) {
            positionChanged = false;
        }
    }

    public void gripPlatform() {
        if (gamepad1.right_trigger > 0) {
            leftPlatformLatcher.setPosition(1);
            rightPlatformLatcher.setPosition(1);
        }
        /*else if (gamepad1.left_trigger > 0) {
            leftPlatformLatcher.setPosition(0);
            rightPlatformLatcher.setPosition(0);
        }*/
        else {
            leftPlatformLatcher.setPosition(0);
            rightPlatformLatcher.setPosition(0);
        }
    }

    public void autoGripBlock() {
        if (!useOneGamepad) {
            if (gamepad1.dpad_right) {
                rightAutoBlockGrabber.setPosition(0); // up
            } else if (gamepad1.dpad_left) {
                rightAutoBlockGrabber.setPosition(1); // down
            }
            if (gamepad1.dpad_down) {
                rightBlockAligner.setPosition(1); // down
            } else if (gamepad1.dpad_up) {
                rightBlockAligner.setPosition(0.2); // up
            }
        }
    }

    public void deployCapstone() {
        if (((gamepad2.a && !gamepad2.start) || (useOneGamepad && gamepad1.a && !gamepad1.start)) && !deployerPositionChanged) {
            time = runtime.time();
            capstoneDeployer.setPosition(deployerClosed ? 1 : 0);
            deployerClosed = !deployerClosed;
            deployerPositionChanged = true;
        }
        else if ((!useOneGamepad && !gamepad2.a) || (useOneGamepad && !gamepad1.a)) {
            deployerPositionChanged = false;
        }

        if (runtime.time() - time > 1.5) {
            capstoneDeployer.setPosition(1);
        }
    }

    public void bringDownLiftAutomated() {
        if (gamepad2.dpad_down || gamepad2.dpad_up || gamepad2.dpad_right || gamepad2.dpad_left) {
            blockClaw.setPosition(.10);
            if (horizontalLimit.getValue() < 1) {
                horizontalSlide.setPower(-.85);
            }
            else {
                horizontalSlide.setPower(0);
            }
            if (lowerVerticalLimit.getValue() < 1) {
                rightVerticalSlide.setPower(-1);
                leftVerticalSlide.setPower(-1);
            }
            else {
                rightVerticalSlide.setPower(0);
                leftVerticalSlide.setPower(0);
            }
        }
    }

    public void useOneGamepad() {
        if ((gamepad1.right_bumper && gamepad1.left_bumper && gamepad1.left_trigger == 1 && gamepad1.right_trigger == 1) || (gamepad2.right_bumper && gamepad2.left_bumper && gamepad2.left_trigger == 1 && gamepad2.right_trigger == 1)) {
            useOneGamepad = !useOneGamepad;
        }
    }

    /*public void launchStaff() {
        if((!useOneGamepad && gamepad1.y && gamepad1.b) || (useOneGamepad && gamepad1.x && gamepad1.y)) {
            staffServo.setPosition(-1);
        }
    }*/

    @Override
    public void stop() {

    }
}