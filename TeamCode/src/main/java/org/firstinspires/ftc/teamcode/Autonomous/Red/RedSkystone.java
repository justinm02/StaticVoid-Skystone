package org.firstinspires.ftc.teamcode.Autonomous.Red;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Autonomous.Auto;


@Autonomous(name = "RedSkystone", group = "Autonomous")
public class RedSkystone extends Auto {

    private double xSkystone[], ySkystone[];
    private double xCrossBridge[], yCrossBridge[];
    private double xPlatform[] = new double[]{0, 16, 16};
    private double yPlatform[] = new double[]{0, 0, -12};

    public void runOpMode() {
        initialize();
        waitForStart();

        try {
            SKYSTONE_POSITION position = determineSkystonePosition("red");

            bringAlignerDown();
            if (position.equals(SKYSTONE_POSITION.MIDDLE)) {
                move(0, 25, -95, .6, .25, .15, "strafe");
            }
            gripBlock();
            pause(1);
            bringAlignerUp();
        }
        catch (InterruptedException e) {
        }
    }
}
