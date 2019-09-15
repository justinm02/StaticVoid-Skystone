package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.Autonomous.PID.PID;
import org.firstinspires.ftc.teamcode.Autonomous.Spline.Bezier;
import org.firstinspires.ftc.teamcode.Autonomous.Spline.Waypoint;

import com.qualcomm.robotcore.util.Range;


@Autonomous(name = "Auto", group = "Autonomous")
public class AutoOp extends LinearOpMode {
    private ElapsedTime runtime;
    private DcMotorEx leftFront, leftBack, rightFront, rightBack, encoderTracker;
    private DcMotorEx[] motors = {leftFront, leftBack, rightFront, rightBack};
    private BNO055IMU imu;
    private PID headingPid = new PID(0.023, 0, 0.0023);
    private PID strafePID = new PID(0, 0, 0); //still need to be determined and tuned
    private final double METERS_TO_INCHES = 39.3701;
    private int initialEncoderPosition;
    private final double WHEEL_CIRCUMFERENCE_IN = Math.PI*3.05; //circumference of parallel deadwheel
    private final int deadWheelTicks = 1440; //tested and validated ticks per revolution of parallel deadwheel
    private final double INCHES_OVER_TICKS = WHEEL_CIRCUMFERENCE_IN/deadWheelTicks; //conversion used to convert inches to ticks (what encoderTracker.getCurrentPosition() reads)

    public void runOpMode() {
        initialize();

        telemetry.addData("Testing github commit", "Success");

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        telemetry.addData("Pos", encoderTracker.getCurrentPosition());
        telemetry.update();
        waitForStart();

        try {
            //(x0, y0), (x1, y1), ..., (x3, y3)
            double xcoords[] = {0.0, -10, 0.0, -10};
            double ycoords[] = {0.0, 0.0, -60, -60};

            initialEncoderPosition = encoderTracker.getCurrentPosition(); //encoder offset from currentPosition
            splineMove(xcoords, ycoords, -0.7); //backwards spline from cube to bridge, ending in same orientation
        }
        catch (InterruptedException e) { }
    }

    public void initialize() {
        initMotors();
        initGyro();

        runtime = new ElapsedTime();
    }

    public void initMotors() {
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftBack = hardwareMap.get(DcMotorEx.class, "leftBack");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        rightBack = hardwareMap.get(DcMotorEx.class, "rightBack");
        encoderTracker = hardwareMap.get(DcMotorEx.class, "encoderTracker");

        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        encoderTracker.setDirection(DcMotor.Direction.REVERSE);

        encoderTracker.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        motors = new DcMotorEx[] {leftFront, leftBack, rightFront, rightBack};
    }

    public void initGyro() {
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imu.initialize(parameters);
    }

    public void move(double targetHeading, double power, String direction) throws InterruptedException {
        int target = 100000; //just a random high value used for only PID testing purposes atm. 100000 ticks has no real significance

        while (Math.abs(initialEncoderPosition - encoderTracker.getCurrentPosition()) < target) {
            correction(power, targetHeading, direction, false);
            heartbeat();
        }

        halt();
    }

    public void strafe(double power, int targetHeading, String direction) throws InterruptedException {
        //current strafe code shown below used solely for testing PID. Not what actual final method will look like (Waiting on hardware to attach a perpendicular deadwheel encoder)
        double currentTime = runtime.time();
        while (runtime.time() - currentTime < 5) {
            telemetry.addData("in loop", 2);
            telemetry.update();
            correction(power, targetHeading, direction, false);
            heartbeat();
        }
    }

    public void turn(double heading, double power, String direction) throws InterruptedException {
        double target = heading;
        double current = currentAngle();

        //leaves room for small error by converting to int as fast turning isn't that accurate at times
        while ((int)heading != (int)currentAngle()) {
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
        if (inverted) {
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
            leftFront.setPower(Range.clip(power + headingPid.getCorrection(current - target, runtime), -1.0, 1.0));
            rightFront.setPower(Range.clip(power - headingPid.getCorrection(current - target, runtime), -1.0, 1.0));
            leftBack.setPower(Range.clip(power + headingPid.getCorrection(current - target, runtime), -1.0, 1.0));
            rightBack.setPower(Range.clip(power - headingPid.getCorrection(current - target, runtime), -1.0, 1.0));
        }

        //pd correction for strafe motion. Right and left are opposites
        else if (movementType.contains("strafe")) {
            if (movementType.contains("left")) {
                leftFront.setPower(-power + headingPid.getCorrection(current - target, runtime));
                rightFront.setPower(power + headingPid.getCorrection(current - target, runtime));
                leftBack.setPower(power - headingPid.getCorrection(current - target, runtime));
                rightBack.setPower(-power - headingPid.getCorrection(current - target, runtime));
            }
            else if (movementType.contains("right")) {
                leftFront.setPower(power - headingPid.getCorrection(current - target, runtime));
                rightFront.setPower(-power - headingPid.getCorrection(current - target, runtime));
                leftBack.setPower(-power + headingPid.getCorrection(current - target, runtime));
                rightBack.setPower(power + headingPid.getCorrection(current - target, runtime));
            }
        }

        telemetry.addData("angle", current);
        telemetry.update();
    }

    public void splineMove(double[] xcoords, double[] ycoords, double power) throws InterruptedException {
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
        while (t<1.0) {
            heartbeat();
            //constantly adjusts heading based on what the current spline angle should be based on the calculated t
            correction(power, (int)(180/Math.PI*spline.getAngle(t, Math.PI/180*lastAngle)), "spline", inverted); //converts lastAngle to radians
            lastAngle = (int)(180/Math.PI*spline.getAngle(t, Math.PI/180*lastAngle)); //converts lastAngle to degrees for telemetry
            //distanceTraveled computed by converting encoderTraveled ticks on deadwheel to inches traveled
            distanceTraveled = INCHES_OVER_TICKS*(encoderTracker.getCurrentPosition()-initialEncoderPosition);
            //t measures progress along curve. Very important for computing splineAngle.
            t = Math.abs(distanceTraveled/arclength);
            telemetry.addData("Distance ", distanceTraveled);
            telemetry.addData("t ", t);
            telemetry.addData("error", currentAngle() - lastAngle);
            telemetry.addData("dxdt", spline.getdxdt(t));
            telemetry.addData("spline angle", (int)(180/Math.PI*spline.getAngle(t, Math.PI/180*lastAngle)));
            telemetry.update();
        }
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
            heartbeat();
        }
    }

    private void heartbeat() throws InterruptedException{
        //if opMode is stopped, will throw and catch an InterruptedException rather than resulting in red text and program crash on phone
        if(!opModeIsActive()) {
            throw new InterruptedException();
        }
    }
}