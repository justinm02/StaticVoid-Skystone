package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

/**
 * Created by Justin Milushev on 9/7/2018.
 */
public class GamerOp extends OpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotorEx leftFront, leftBack, rightFront, rightBack, slide;
    private Servo frontPlatformLatcher, backPlatformLatcher;
    private boolean precision, direction;
    private boolean canTogglePrecision, canToggleDirection, strafeMode;
    private int baseSlidePosition;

    @Override
    public void init() {
        //after driver hits init
        setUpDriveTrain();
        setUpSlide();
        setUpServos();

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

    public void setUpSlide() {
        slide = (DcMotorEx) hardwareMap.dcMotor.get("slide");
        slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        baseSlidePosition = slide.getCurrentPosition();
    }

    public void setUpServos() {
        //frontPlatformLatcher = hardwareMap.servo.get("frontLatcher");
        //backPlatformLatcher = hardwareMap.servo.get("backLatcher");
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
        driveBot();
        moveSlide();
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

        if (gamepad1.right_bumper)
            strafeMode = !strafeMode;

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

        //If (?) precision, set up to .2 of power. Else (:) 1.0 of power
        leftFront.setPower(leftFrontPower * (precision ? 0.2 : 1.0));
        leftBack.setPower(leftRearPower * (precision ? 0.2 : 1.0));
        rightFront.setPower(rightFrontPower * (precision ? 0.2 : 1.0));
        rightBack.setPower(rightRearPower * (precision ? 0.2 : 1.0));

        telemetry.addData("Front Motors", "Left Front (%.2f), Right Front (%.2f)", leftFrontPower, rightFrontPower);
        telemetry.addData("Rear Motors", "Left Rear (%.2f), Right Rear (%.2f)", leftRearPower, rightRearPower);
    }

    public void moveSlide() {
        if (slide.getCurrentPosition() <= baseSlidePosition + 200 || slide.getCurrentPosition() >= baseSlidePosition - 500) {
        }

        slide.setPower(gamepad1.right_stick_y);
        telemetry.addData("slide power", slide.getPower());
        telemetry.addData("slide position", slide.getCurrentPosition());
        telemetry.update();
    }

    @Override
    public void stop() {

    }
}
