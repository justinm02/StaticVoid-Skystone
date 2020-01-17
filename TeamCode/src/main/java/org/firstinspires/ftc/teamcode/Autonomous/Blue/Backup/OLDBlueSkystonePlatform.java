package org.firstinspires.ftc.teamcode.Autonomous.Blue.Backup;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.Autonomous.Auto;


@Autonomous(name = "OLDBlueSkystonePlatform", group = "Autonomous")
@Disabled
public class OLDBlueSkystonePlatform extends Auto {

    private double xSkystone[], ySkystone[];
    private double xCrossBridge[], yCrossBridge[];
    private double xPlatform[] = new double[]{0, 16, 16};
    private double yPlatform[] = new double[]{0, 0, -12};

    public void runOpMode() {
        initialize();
        waitForStart();

        try {
            SKYSTONE_POSITION position = determineSkystonePosition("blue");
            //pause(1);

            telemetry.addData("position", position);
            telemetry.update();
            //intake();

            int getPlatform = 0;

            move(0, 4, -90, .5, "strafe");
            PIDTurn(180, 1);

            if (position.equals(SKYSTONE_POSITION.RIGHT)) {
                /*xSkystone = new double[]{0, 0, -12};
                ySkystone = new double[]{0, -80, -80};*/
                getPlatform = 37;

                move(180, 33, -81, .725, "strafe");

                intake();
                pause(.5);
                move(180, 2, 180, .725, "straight");
                pause(2);
            }
            else if (position.equals(SKYSTONE_POSITION.MIDDLE)) {
                /*xSkystone = new double[]{0, 0, -12};
                ySkystone = new double[]{0, -96, -96};*/
                getPlatform = 45;

                move(-180, 36, -94, .725, "strafe");

                intake();
                pause(.5);
                move(-180, 2, -180, .725, "straight");
                pause(2);
            }
            else {
                /*xSkystone = new double[]{0, 0, -12};
                ySkystone = new double[]{0, -88, -88};*/
                getPlatform = 41;

                move(180, 34, -91, .725, "strafe");
                intake();
                pause(.5);
                move(180, 2, 180, .725, "straight");
                pause(2);
            }
            stopIntake();

            gripBlock();
            move(180, 16, 90, .725, "strafe");

            move(180, getPlatform, 0, .725, "straight");
            PIDTurn(90, 1);
            move(90, 6, -90, .5, "straight");

            gripPlatform();
            pause(.5);
            move(90, 7, 90, .725, "straight");
            PIDTurn(180, .6);
            releasePlatform();
            //pause(.5);
            slideAndMoveByTime(3, -.3);

            //extendHorizontalSlide();
            releaseBlock();
            extendVerticalLift();
            move(180, 4, 180, .725, "straight");
            retractVerticalLift();
            move(180, 2, -90, .725, "strafe");
            move(180, 15, 180, .725, "straight");
            retractHorizontalSlide();
        }
        catch (InterruptedException e) {
        }
    }
}
