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
    private DcMotorEx leftFront, leftBack, rightFront, rightBack, rightIntake, leftIntake, verticalSlide, parallelRightEncoderTracker;
    private Servo blockClaw, blockRotator, leftPlatformLatcher, rightPlatformLatcher;
    private CRServo horizontalSlide;
    private TouchSensor horizontalLimit, lowerVerticalLimit;
    private PositionTracker positionTracker = new PositionTracker(0, 0, 0);
    private boolean precision, direction;
    private boolean canTogglePrecision, canToggleDirection, strafeMode, precisionChanged, directionChanged;
    private boolean useOneGamepad;
    private int verticalSlideLimit; //may be touchSensor eventually
    private boolean positionChanged = false;
    private boolean closed = false;
    private double baseParallelRightPosition;
    private double baseParallelLeftPosition;
    private double basePerpendicularPosition;
    private final int deadWheelTicks = 4096;
    private final double WHEEL_CIRCUMFERENCE_IN = Math.PI*3.05; //circumference of parallel deadwheel
    private final double DEADWHEEL_INCHES_OVER_TICKS = WHEEL_CIRCUMFERENCE_IN/deadWheelTicks;

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
        strafeMode = false;
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
        horizontalSlide.setDirection(DcMotorSimple.Direction.REVERSE);

        horizontalLimit = hardwareMap.touchSensor.get("horizontalLimit");
        lowerVerticalLimit = hardwareMap.touchSensor.get("lowerVerticalLimit");
    }

    public void setUpServos() {
        blockClaw = hardwareMap.servo.get("blockClaw");
        blockRotator = hardwareMap.servo.get("blockRotator");
        leftPlatformLatcher = hardwareMap.servo.get("leftPlatformLatcher");
        rightPlatformLatcher = hardwareMap.servo.get("rightPlatformLatcher");

        blockClaw.setPosition(.26);
        blockRotator.setPosition(.485);

        leftPlatformLatcher.setPosition(0);
        rightPlatformLatcher.setPosition(1);
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

        useEncoders();
        driveBot();
        intake();
        moveSlides();
        gripBlock();
        gripPlatform();
        rotateBlock();

        bringDownLiftAutomated();

        useOneGamepad();
    }

    public void useEncoders() {
        double parallelLeftTicks = (leftIntake.getCurrentPosition() - baseParallelLeftPosition);
        double parallelRightTicks = parallelRightEncoderTracker.getCurrentPosition() - baseParallelRightPosition;
        double perpendicularTicks = rightIntake.getCurrentPosition() - basePerpendicularPosition;

        positionTracker.updateTicks(parallelLeftTicks, parallelRightTicks, perpendicularTicks);
        if(Math.abs(gamepad1.left_stick_x) < Math.cos(Math.toRadians(5)) || Math.sqrt(Math.pow(gamepad1.left_stick_x,2) + Math.pow(gamepad1.left_stick_y,2)) < .1 ) {
            //positionTracker.updateLocationAndPose("", telemetry);
        }
        else {
            //positionTracker.updateLocationAndPose("strafe", telemetry);
        }


        telemetry.addData("X: ", positionTracker.getCurrentX());
        telemetry.addData("Y: ", positionTracker.getCurrentY());
        telemetry.addData("Current Angle: ", positionTracker.getCurrentAngle());
        telemetry.addData("parallel left ticks", parallelLeftTicks);
        telemetry.addData("parallel right ticks", parallelRightTicks);
        telemetry.addData("perpendicular ticks", perpendicularTicks);
        telemetry.addData("Distance traveled", (parallelLeftTicks + parallelLeftTicks)*DEADWHEEL_INCHES_OVER_TICKS/2);
        telemetry.update();
    }

    public void gripBlock() {
        /*if ((gamepad2.b  || (useOneGamepad && gamepad1.b)) && !closed && !gamepad2.start) {
            blockClaw.setPosition(0);
            closed = !closed;
        }
        else if((!gamepad2.b || (useOneGamepad && gamepad1.b)) && closed && !gamepad2.start) {
            blockClaw.setPosition(1);
            closed = !closed;
        }*/

        if ((gamepad2.b || (useOneGamepad && gamepad1.b)) && !gamepad2.start && !positionChanged) {
            blockClaw.setPosition(closed ? 1 : 0.26);
            closed = !closed;
            positionChanged = true;
        }
        else if (!gamepad2.b) {
            positionChanged = false;
        }

        //telemetry.addData("block claw pos", blockClaw.getPosition());
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


        double leftFrontPower = xLinear * Math.sin(joystickAngle) - xTurn;
        double rightFrontPower = xLinear * Math.cos(joystickAngle) + xTurn;
        double leftBackPower = xLinear * Math.cos(joystickAngle) - xTurn;
        double rightBackPower = xLinear * Math.sin(joystickAngle) + xTurn;

        double[] motorPowers = new double[]{leftFrontPower, rightFrontPower, leftBackPower, rightBackPower};
        convertMotorPowers(motorPowers, xLinear, xTurn);

        leftFront.setPower(precision ? 0.4 * motorPowers[0] : motorPowers[0]);
        rightFront.setPower(precision ? 0.4 * motorPowers[1] : motorPowers[1]);
        leftBack.setPower(precision ? 0.4 * motorPowers[2] : motorPowers[2]);
        rightBack.setPower(precision ? 0.4 * motorPowers[3] : motorPowers[3]);

        telemetry.addData("Front Motors", "Left Front (%.2f), Right Front (%.2f)", leftFront.getPower(), rightFront.getPower());
        telemetry.addData("joystickAngle", joystickAngle);
        telemetry.addData("Rear Motors", "Left Rear (%.2f), Right Rear (%.2f)", leftBack.getPower(), rightBack.getPower());
    }

    public void convertMotorPowers(double[] motorPowers, double xLinear, double xTurn) {
        double maxPower = getMaxMagnitude(motorPowers);

        double conversion = Math.abs(Math.sqrt((Math.pow(xLinear, 2) + Math.pow(xTurn, 2)) / 2) / maxPower);

        for (int i = 0; i < motorPowers.length; i++) {
            motorPowers[i] *= conversion;
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

        /*telemetry.addData("leftIntake pos", leftIntake.getCurrentPosition());
        telemetry.addData("rightIntake pos", rightIntake.getCurrentPosition());
        telemetry.update();*/
    }

    public void moveSlides() {
        /*if (gamepad2.dpad_up || (useOneGamepad && gamepad1.dpad_up)) {
            verticalSlide.setPower(1);
        }
        else if (gamepad2.dpad_down || (useOneGamepad && gamepad1.dpad_down)) {
            verticalSlide.setPower(-1);
        }
        else
            verticalSlide.setPower(0);*/

        /*if (verticalSlide.getCurrentPosition() < verticalSlideLimit || gamepad2.left_stick_y > 0) {*/
        if(lowerVerticalLimit.getValue() < 1 || gamepad2.left_stick_y < 0) {
            verticalSlide.setPower(-gamepad2.left_stick_y);
        }
        else {
            verticalSlide.setPower(0);
        }
        if (lowerVerticalLimit.getValue() == 0 && gamepad2.left_stick_y == 0) {
            verticalSlide.setPower(.12);
        }
        /*}
        else {
            verticalSlide.setPower(0);
        }*/
        if (horizontalLimit.getValue() < 1 || gamepad2.right_stick_y < 0) {
            horizontalSlide.setPower(gamepad2.right_stick_y);
        }
        else {
            horizontalSlide.setPower(0);
        }

        /*telemetry.addData("slide power", verticalSlide.getPower());
        telemetry.addData("slide position", verticalSlide.getCurrentPosition());
        telemetry.addData("horizontal limit", horizontalLimit.getValue());
        telemetry.update();*/
    }

    public void rotateBlock() {
        //values will be adjusted
        if (gamepad2.x) {
            blockRotator.setPosition(0.485); //return to normal
        }
        else if (gamepad2.y) {
            blockRotator.setPosition(.9);
        }
        else if (gamepad2.a) {
            blockRotator.setPosition(0);
        }
    }

    public void gripPlatform() {
        if (gamepad1.right_trigger > 0) {
            leftPlatformLatcher.setPosition(0);
            rightPlatformLatcher.setPosition(1);
        }
        else if (gamepad1.left_trigger > 0) {
            leftPlatformLatcher.setPosition(.625);
            rightPlatformLatcher.setPosition(.275);
        }
    }

    public void bringDownLiftAutomated() {
        if ((gamepad2.dpad_down || gamepad2.dpad_up || gamepad2.dpad_right || gamepad2.dpad_left) && (0.25 < blockRotator.getPosition() && blockRotator.getPosition() < 0.75)) {
            blockClaw.setPosition(.26);
            blockRotator.setPosition(.485);
            if (horizontalLimit.getValue() < 1) {
                horizontalSlide.setPower(1);
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
