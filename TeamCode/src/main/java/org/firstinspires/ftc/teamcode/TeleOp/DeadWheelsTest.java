package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Localization.PositionTracker;

public class DeadWheelsTest extends OpMode {
    private DcMotorEx leftFront, leftBack, rightFront, rightBack, parallelRightEncoderTracker, leftIntake, rightIntake, verticalSlide;
    //private CRServo tapeMeasure;
    private ElapsedTime runtime = new ElapsedTime();
    private boolean precision, direction;
    private boolean canTogglePrecision, canToggleDirection, strafeMode;
    private final double WHEEL_CIRCUMFERENCE_IN = Math.PI*3.05;
    private final double DEADWHEEL_INCHES_OVER_TICKS = WHEEL_CIRCUMFERENCE_IN/4096;
    private double parallelLeftStartingPosition, parallelRightStartingPosition, perpendicularStartingPosition;
    private PositionTracker positionTracker = new PositionTracker(0, 0, 0);

    @Override
    public void init() {
        //after driver hits init
        setUpDriveTrain();
        setUpIntake();
        setUpTapeMeasure();

        /*parallelLeftEncoderTracker = hardwareMap.get(DcMotorEx.class, "parallelLeftEncoderTracker");
        parallelLeftEncoderTracker.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        parallelLeftEncoderTracker.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);*/

        parallelRightEncoderTracker = hardwareMap.get(DcMotorEx.class, "perpendicularEncoderTracker");
        parallelRightEncoderTracker.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        parallelRightEncoderTracker.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        parallelLeftStartingPosition = leftIntake.getCurrentPosition();
        parallelRightStartingPosition = rightIntake.getCurrentPosition();
        perpendicularStartingPosition = parallelRightEncoderTracker.getCurrentPosition();

        precision = false;
        direction = false;
        strafeMode = false;

        verticalSlide = (DcMotorEx) hardwareMap.dcMotor.get("verticalSlide");
        verticalSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        verticalSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        verticalSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        verticalSlide.setDirection(DcMotor.Direction.REVERSE);

        telemetry.addData("Status", "Initialized");
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
        leftIntake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftIntake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftIntake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rightIntake = (DcMotorEx) hardwareMap.dcMotor.get("rightIntake");
        rightIntake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightIntake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightIntake.setDirection(DcMotor.Direction.REVERSE);
    }

    public void setUpTapeMeasure() {
        //tapeMeasure = (CRServo) hardwareMap.servo.get("tapeMeasure");
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
        double parallelLeftTicks = leftIntake.getCurrentPosition();
        double parallelRightTicks = parallelRightEncoderTracker.getCurrentPosition();
        double perpendicularTicks = rightIntake.getCurrentPosition();

        positionTracker.updateTicks(parallelLeftTicks, parallelRightTicks, perpendicularTicks);
        //positionTracker.updateLocationAndPose("strafe", telemetry);


        telemetry.addData("X: ", positionTracker.getCurrentX());
        telemetry.addData("Y: ", positionTracker.getCurrentY());
        telemetry.addData("Current Angle: ", positionTracker.getCurrentAngle());
        telemetry.addData("parallel left ticks", leftIntake.getCurrentPosition());
        telemetry.addData("parallel right ticks", parallelRightEncoderTracker.getCurrentPosition());
        telemetry.addData("perpendicular ticks", rightIntake.getCurrentPosition());
        telemetry.update();

        verticalSlide.setPower(gamepad2.right_stick_y);

        driveBot();
        intake();
        useTapeMeasure();
    }

    public void driveBot() {
        // Precision mode toggled by pressing right stick
        if (gamepad1.right_stick_button && canTogglePrecision) {
            precision = !precision;
            canTogglePrecision = false;
        }
        else if (!gamepad1.right_stick_button)
            canTogglePrecision = true;

        if (gamepad1.left_stick_button && canToggleDirection) {
            direction = !direction;
            canTogglePrecision = false;
        }
        else if (!gamepad1.left_stick_button)
            canToggleDirection = true;

        double x = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y);
        double stickAngle = Math.atan2(direction ? -gamepad1.left_stick_y : gamepad1.left_stick_y, direction ? gamepad1.left_stick_x : -gamepad1.left_stick_x); // desired robot angle from the angle of stick

        double powerAngle = 0;

        if (!strafeMode)
            powerAngle = stickAngle - (Math.PI / 4); //conversion for correct power values
        else
            powerAngle = stickAngle - (3 * Math.PI / 4); //conversion for correct power values

        double rightX = gamepad1.right_stick_x;

        final double leftFrontPower = Range.clip(x * Math.cos(powerAngle) - rightX, -1.0, 1.0);
        final double leftRearPower = Range.clip(x * Math.sin(powerAngle) - rightX, -1.0, 1.0);
        final double rightFrontPower = Range.clip(x * Math.sin(powerAngle) + rightX, -1.0, 1.0);
        final double rightRearPower = Range.clip(x * Math.cos(powerAngle) + rightX, -1.0, 1.0);

        //If (?) precision, set up to .4 of power. Else (:) 1.0 of power
        leftFront.setPower(leftFrontPower * (precision ? 0.4 : 1.0));
        leftBack.setPower(leftRearPower * (precision ? 0.4 : 1.0));
        rightFront.setPower(rightFrontPower * (precision ? 0.4 : 1.0));
        rightBack.setPower(rightRearPower * (precision ? 0.4 : 1.0));

        telemetry.addData("Front Motors", "Left Front (%.2f), Right Front (%.2f)", leftFrontPower, rightFrontPower);
        telemetry.addData("Rear Motors", "Left Rear (%.2f), Right Rear (%.2f)", leftRearPower, rightRearPower);
        telemetry.addData("Front Motors Power", "Left Front (%.2f), Right Front (%.2f)", leftFront.getPower(), rightFront.getPower());
        telemetry.addData("Rear Motors Power", "Left Rear (%.2f), Right Rear (%.2f)", leftBack.getPower(), rightBack.getPower());

        telemetry.update();
    }

    public void intake() {
        if (gamepad1.a) {
            leftIntake.setPower(1);
            rightIntake.setPower(1);
        }
        else if (gamepad1.b) {
            leftIntake.setPower(-1);
            rightIntake.setPower(-1);
        }
        else {
            leftIntake.setPower(0);
            rightIntake.setPower(0);
        }
    }

    public void useTapeMeasure() {
        if (gamepad1.dpad_down) {
            //tapeMeasure.setPower(-1);
        }
        else if (gamepad1.dpad_up) {
            //tapeMeasure.setPower(1);
        }
        else {
            //tapeMeasure.setPower(0);
        }
    }
}
