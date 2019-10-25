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

            strafe(.3, 0, "straferight", 8);

            adjustClaw();
            pause(1.25);
            moveSlideByTime();

            double xcoords2[];
            double ycoords2[];

            double xcoords3[];
            double ycoords3[];

            double xcoords4[];
            double ycoords4[];

            SKYSTONE_POSITION position = determineSkystonePlacement();

            if (position.equals(SKYSTONE_POSITION.RIGHT)) {
                xcoords2 = new double[]{0, 12, 12};
                ycoords2 = new double[]{0, 0, -6};

                xcoords3 = new double[]{0, 16, 3, 3};
                ycoords3 = new double[]{0, -15, -47, -60};

                xcoords4 = new double[]{0, -44, -44};
                ycoords4 = new double[]{0, 0, -12};

            }

            else if (position.equals(SKYSTONE_POSITION.MIDDLE)) {
                xcoords2 = new double[]{0, 23, 23};
                ycoords2 = new double[]{0, 0, -8};

                xcoords3 = new double[]{0, 16, 3, 3};
                ycoords3 = new double[]{0, -15, -47, -70};

                xcoords4 = new double[]{0, -56, -56};
                ycoords4 = new double[]{0, 0, -12};
            }

            else {
                move(0, -.3, 7, "straight");

                xcoords2 = new double[]{0, 11, 11};
                ycoords2 = new double[]{0, 0, -8};

                xcoords3 = new double[]{0, 16, 3, 3};
                ycoords3 = new double[]{0, -15, -47, -55};

                xcoords4 = new double[]{0, -56, -56};
                ycoords4 = new double[]{0, 0, -12};
            }

            splineMove(xcoords2, ycoords2, .25, 0); //backwards spline from cube to bridge, ending in same orientation

            move(-90, .3, 18, "straight");

            gripBlock();
            pause(1);

            move(-90, -.3, 32, "straight");

            splineMove(xcoords3, ycoords3, .35, -Math.PI/2);
        }
        catch (InterruptedException e) { }
    }
}