package org.firstinspires.ftc.teamcode.Autonomous.Red;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Autonomous.Auto;


@Autonomous(name = "RedSkystone", group = "Autonomous")
public class RedSkystone extends Auto {

    private double xSkystone[], ySkystone[];

    public void runOpMode() {
        initialize();
        waitForStart();

        try {
            SKYSTONE_POSITION position = determineSkystonePosition("red");

            bringAlignerDown();
            if (position.equals(SKYSTONE_POSITION.MIDDLE)) {
                move(0, 27, -90, .5, .25, 0, "strafe");
                xSkystone = new double[]{0, -75, -10, -85};
                ySkystone = new double[]{0, 0, -3, -3};
            }
            gripBlock();
            pause(1);
            bringAlignerUp();
            pause(1);

            move(0, 5, 90, .35, .25, 0, "strafe");
            splineMove(xSkystone, ySkystone, -.8, -.15, 0, 0);
        }
        catch (InterruptedException e) {
        }
    }
}