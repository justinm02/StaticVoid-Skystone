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

            double xcoords2[] = {0, 40, 40};
            double ycoords2[] = {0, 0, -40};

            //strafe(.3, 0, "straferight", 200);

            splineMove(xcoords2, ycoords2, .1); //backwards spline from cube to bridge, ending in same orientation
        }
        catch (InterruptedException e) { }
    }
}