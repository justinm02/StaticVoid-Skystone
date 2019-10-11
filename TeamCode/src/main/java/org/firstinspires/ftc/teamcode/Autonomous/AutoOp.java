package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Auto", group = "Autonomous")
public class AutoOp extends Auto {

    public void runOpMode() {
        initialize();

        waitForStart();

        try {
            //(x0, y0), (x1, y1), ..., (x3, y3)
            double xcoords1[] = {0.0, 10, 0.0, 10};
            double ycoords1[] = {0.0, 0.0, 60, 60};

            //move(0, .25, "straight");

            double xcoords2[] = new double[]{0, 6, 6};
            double ycoords2[] = new double[]{0, 0, -8};

            if (determineSkystonePlacement().equals(SKYSTONE_POSITION.LEFT)) {
                xcoords2 = new double[]{0, 14, 14};
                ycoords2 = new double[]{0, 0, -8};
            }
            else if (determineSkystonePlacement().equals(SKYSTONE_POSITION.MIDDLE)) {
                xcoords2 = new double[]{0, 14, 14};
                ycoords2 = new double[]{0, 0, -8};
            }

            //strafe(.3, 0, "straferight", 200);
            pause(.25);

            splineMove(xcoords2, ycoords2, .25); //backwards spline from cube to bridge, ending in same orientation

            move(-90, .3, 7, "straight");


        }
        catch (InterruptedException e) { }
    }
}