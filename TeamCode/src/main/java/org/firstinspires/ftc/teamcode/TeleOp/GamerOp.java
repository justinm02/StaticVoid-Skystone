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

public class GamerOp extends OpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotorEx leftFront, leftBack, rightFront, rightBack, rightIntake, leftIntake, verticalSlide, parallelRightEncoderTracker;
    private Servo blockClaw, leftPlatformLatcher, rightPlatformLatcher, autoBlockGrabber, blockAligner, capstoneDeployer;
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

    @Override
    public void init() {
        //after driver hits init
        setUpDriveTrain();
        setUpIntake();
        setUpSlides();
        setUpServos();

        parallelRightEncoderTracker = (DcMotorEx) hardwareMap.dcMotor.get("parallelRightEncoderTracker");
        parallelRightEncoderTracker.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        precision = false;
        direction = false;
        useOneGamepad = false;
        precisionChanged = false;
        directionChanged = false;

        closed = false;
        positionChanged = false;

        telemetry.addData("Status", "Initialized");

        baseParallelLeftPosition = leftIntake.getCurrentPosition();
        baseParallelRightPosition = parallelRightEncoderTracker.getCurrentPosition();
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
        leftIntake.setDirection(DcMotor.Direction.REVERSE);


        rightIntake = (DcMotorEx) hardwareMap.dcMotor.get("rightIntake");
        rightIntake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightIntake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightIntake.setDirection(DcMotor.Direction.REVERSE);
    }

    public void setUpSlides() {
        verticalSlide = (DcMotorEx) hardwareMap.dcMotor.get("verticalSlide");
        verticalSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        verticalSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //verticalSlide.setDirection(DcMotor.Direction.REVERSE);

        horizontalSlide = hardwareMap.get(CRServo.class, "horizontalSlide");
        //horizontalSlide.setDirection(DcMotorSimple.Direction.REVERSE);

        horizontalLimit = hardwareMap.touchSensor.get("horizontalLimit");
        lowerVerticalLimit = hardwareMap.touchSensor.get("lowerVerticalLimit");
    }

    public void setUpServos() {
        blockClaw = hardwareMap.servo.get("blockClaw");
        leftPlatformLatcher = hardwareMap.servo.get("leftPlatformLatcher");
        rightPlatformLatcher = hardwareMap.servo.get("rightPlatformLatcher");
        autoBlockGrabber = hardwareMap.servo.get("autoBlockGrabber");
        blockAligner = hardwareMap.servo.get("blockAligner");
        capstoneDeployer = hardwareMap.servo.get("capstoneDeployer");

        blockClaw.setPosition(.26);

        leftPlatformLatcher.setPosition(0);
        rightPlatformLatcher.setPosition(0);
        autoBlockGrabber.setPosition(0);
        blockAligner.setPosition(.2);
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

        bringDownLiftAutomated();

        useOneGamepad();
    }

    public void useEncoders() {
        double parallelLeftTicks = (leftIntake.getCurrentPosition() - baseParallelLeftPosition);
        double parallelRightTicks = parallelRightEncoderTracker.getCurrentPosition() - baseParallelRightPosition;
        double perpendicularTicks = rightIntake.getCurrentPosition() - basePerpendicularPosition;

        if(Math.abs(gamepad1.left_stick_x) < Math.cos(Math.toRadians(5)) || Math.sqrt(Math.pow(gamepad1.left_stick_x,2) + Math.pow(gamepad1.left_stick_y,2)) < .1 ) {
            //positionTracker.updateLocationAndPose("", telemetry);
        }
        else {
            //positionTracker.updateLocationAndPose("strafe", telemetry);
        }


        /*telemetry.addData("X: ", positionTracker.getCurrentX());
        telemetry.addData("Y: ", positionTracker.getCurrentY());
        telemetry.addData("Current Angle: ", positionTracker.getCurrentAngle());
        telemetry.addData("parallel left ticks", parallelLeftTicks);
        telemetry.addData("parallel right ticks", parallelRightTicks);
        telemetry.addData("perpendicular ticks", perpendicularTicks);
        telemetry.addData("Distance traveled", (parallelLeftTicks + parallelLeftTicks)*DEADWHEEL_INCHES_OVER_TICKS/2);*/
        telemetry.update();
    }

    public void driveBot() {
        if (gamepad1.a && !precisionChanged) {
            precision = !precision;
            precisionChanged = true;
        }
        else if (!gamepad1.a) {
            precisionChanged = false;
        }

        if (gamepad1.x && !directionChanged) {
            direction = !direction;
            directionChanged = true;
        }
        else if (!gamepad1.x) {
            directionChanged = false;
        }

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

        telemetry.addData("Front Motors", "Left Front (%.2f), Right Front (%.2f)", leftFront.getPower(), rightFront.getPower());
        telemetry.addData("joystickAngle", joystickAngle);
        telemetry.addData("Rear Motors", "Left Rear (%.2f), Right Rear (%.2f)", leftBack.getPower(), rightBack.getPower());
    }

    public double[] convertMotorPowers(double[] motorPowers, double xLinear, double xTurn) {
        double maxPower = getMaxMagnitude(motorPowers);

        double conversion = Math.abs(Math.sqrt((Math.pow(xLinear,2) + Math.pow(xTurn, 2))/*/2*/)/maxPower);

        telemetry.addData("maxPower", maxPower);
        telemetry.addData("conversion", conversion);
        telemetry.update();

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
        if(lowerVerticalLimit.getValue() < 1 || gamepad2.left_stick_y < 0) {
            verticalSlide.setPower(-gamepad2.left_stick_y);
        }
        else {
            verticalSlide.setPower(0);
        }
        if (lowerVerticalLimit.getValue() == 0 && gamepad2.left_stick_y == 0) {
            verticalSlide.setPower(.12);
        }

        if (horizontalLimit.getValue() < 1 || gamepad2.right_stick_y < 0) {
            horizontalSlide.setPower(Range.clip(gamepad2.right_stick_y, -.85, .85));
        }
        else {
            horizontalSlide.setPower(0);
        }

        /*telemetry.addData("slide power", verticalSlide.getPower());
        telemetry.addData("slide position", verticalSlide.getCurrentPosition());
        telemetry.addData("horizontal limit", horizontalLimit.getValue());
        telemetry.update();*/
    }

    public void gripBlock() {
        if ((gamepad2.b || (useOneGamepad && gamepad1.b)) && !gamepad2.start && !positionChanged) {
            blockClaw.setPosition(closed ? 1 : 0.26);
            closed = !closed;
            positionChanged = true;
        }
        else if (!gamepad2.b) {
            positionChanged = false;
        }
    }

    public void gripPlatform() {
        if (gamepad1.right_trigger > 0) {
            leftPlatformLatcher.setPosition(1);
            rightPlatformLatcher.setPosition(1);
        }
        else if (gamepad1.left_trigger > 0) {
            leftPlatformLatcher.setPosition(0);
            rightPlatformLatcher.setPosition(0);
        }
    }

    public void autoGripBlock() {
        if (gamepad1.dpad_up) {
            autoBlockGrabber.setPosition(0); // up
        }
        else if (gamepad1.dpad_down) {
            autoBlockGrabber.setPosition(1); // down
        }
        if (gamepad1.dpad_left) {
            blockAligner.setPosition(1); // down
        }
        else if (gamepad1.dpad_right) {
            blockAligner.setPosition(0.2); // up
        }
    }

    public void deployCapstone() {
        if (gamepad2.a && !gamepad2.start && !deployerPositionChanged) {
            time = runtime.time();
            capstoneDeployer.setPosition(deployerClosed ? 1 : 0);
            deployerClosed = !deployerClosed;
            deployerPositionChanged = true;
        }
        else if (!gamepad2.a) {
            deployerPositionChanged = false;
        }
        if (runtime.time() - time > 1.5) {
            capstoneDeployer.setPosition(1);
        }
    }

    public void bringDownLiftAutomated() {
        if (gamepad2.dpad_down || gamepad2.dpad_up || gamepad2.dpad_right || gamepad2.dpad_left) {
            blockClaw.setPosition(.26);
            if (horizontalLimit.getValue() < 1) {
                horizontalSlide.setPower(.85);
            }
            else {
                horizontalSlide.setPower(0);
            }
            if (lowerVerticalLimit.getValue() < 1) {
                verticalSlide.setPower(-1);
            }
            else {
                verticalSlide.setPower(0);
            }
        }
    }

    public void useOneGamepad() {
        if ((gamepad1.right_bumper && gamepad1.left_bumper && gamepad1.left_trigger == 1 && gamepad1.right_trigger == 1) || (gamepad2.right_bumper && gamepad2.left_bumper && gamepad2.left_trigger == 1 && gamepad2.right_trigger == 1)) {
            useOneGamepad = !useOneGamepad;
        }
    }

    @Override
    public void stop() {

    }
}