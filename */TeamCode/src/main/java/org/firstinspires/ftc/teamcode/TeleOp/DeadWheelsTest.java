package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Localization.PositionTracker;

public class DeadWheelsTest extends OpMode {
    private DcMotorEx leftFront, leftBack, rightFront, rightBack, parallelLeftEncoderTracker, parallelRightEncoderTracker, perpendicularEncoderTracker;
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

        parallelLeftEncoderTracker = hardwareMap.get(DcMotorEx.class, "parallelLeftEncoderTracker");
        parallelLeftEncoderTracker.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        parallelLeftEncoderTracker.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        parallelRightEncoderTracker = hardwareMap.get(DcMotorEx.class, "parallelRightEncoderTracker");
        parallelRightEncoderTracker.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        parallelRightEncoderTracker.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        perpendicularEncoderTracker = hardwareMap.get(DcMotorEx.class, "perpendicularEncoderTracker");
        perpendicularEncoderTracker.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        perpendicularEncoderTracker.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        parallelLeftStartingPosition = parallelLeftEncoderTracker.getCurrentPosition();
        parallelRightStartingPosition = parallelRightEncoderTracker.getCurrentPosition();
        perpendicularStartingPosition = perpendicularEncoderTracker.getCurrentPosition();

        precision = false;
        direction = false;
        strafeMode = false;

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
        double parallelLeftTicks = parallelLeftEncoderTracker.getCurrentPosition();
        double parallelRightTicks = parallelRightEncoderTracker.getCurrentPosition();
        double perpendicularTicks = perpendicularEncoderTracker.getCurrentPosition();

        positionTracker.updateTicks(parallelLeftTicks, parallelRightTicks, perpendicularTicks);
        positionTracker.updateLocationAndPose("normal");


        telemetry.addData("X: ", positionTracker.getCurrentX());
        telemetry.addData("Y: ", positionTracker.getCurrentY());
        telemetry.addData("Current Angle: ", positionTracker.getCurrentAngle());
        telemetry.addData("parallel left ticks", parallelLeftTicks);
        telemetry.addData("parallel right ticks", parallelRightTicks);
        telemetry.update();

        driveBot();
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
}
