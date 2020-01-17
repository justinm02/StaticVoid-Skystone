package org.firstinspires.ftc.teamcode.Autonomous.Red.Backup;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.Autonomous.Auto;


@Autonomous(name = "OLDRedSkystone", group = "Autonomous")
@Disabled
public class OLDRedSkystone extends Auto {

    private double xSkystone[], ySkystone[];
    private double xCrossBridge[], yCrossBridge[];
    private double xPlatform[] = new double[]{0, 16, 16};
    private double yPlatform[] = new double[]{0, 0, -12};

    public void runOpMode() {
        initialize();
        waitForStart();

        try {
            SKYSTONE_POSITION position = determineSkystonePosition("red");
            //pause(1);

            telemetry.addData("position", position);
            telemetry.update();
            //intake();
            int getPlatform = 0;

            if (position.equals(SKYSTONE_POSITION.LEFT)) {
                /*xSkystone = new double[]{0, 0, 12};
                ySkystone = new double[]{0, -80, -80};*/
                getPlatform = 36;

                move(0, 39, -95, .725, "strafe");
                intake();
                pause(.5);
                move(0, 2, 0, .725, "straight");
                pause(2);
            }
            else if (position.equals(SKYSTONE_POSITION.MIDDLE)) {
                /*xSkystone = new double[]{0, 0, 12};
                ySkystone = new double[]{0, -96, -96};*/
                getPlatform = 44;

                move(0, 40, -77, .725, "strafe");
                intake();
                pause(.5);
                move(0, 2, 0, .725, "straight");
                pause(2);
            }
            else {
                /*xSkystone = new double[]{0, 0, 12};
                ySkystone = new double[]{0, -88, -88};*/
                getPlatform = 40;

                move(0, 39, -84, .725, "strafe");
                intake();
                pause(.5);
                move(0, 2, 0, .725, "straight");
                pause(2);


            }
            stopIntake();

            gripBlock();
            move(0, 16, 90, .725, "strafe");

            move(0, getPlatform, 180, .725, "straight");
            PIDTurn(90, 1);
            move(90, 6, -90, .725, "straight");

            gripPlatform();
            pause(1);
            move(90, 7, 90, .725, "straight");
            PIDTurn(0, .6);
            releasePlatform();
            //pause(.5);
            slideAndMoveByTime(3, -.3);

            //extendHorizontalSlide();
            releaseBlock();
            extendVerticalLift();
            move(0, 4, 0, .725, "straight");
            retractVerticalLift();
            move(0, 15, 0, .725, "straight");
            retractHorizontalSlide();


        }
        catch (InterruptedException e) {
        }
    }
}
