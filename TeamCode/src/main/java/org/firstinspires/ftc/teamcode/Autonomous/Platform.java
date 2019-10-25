package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Platform", group = "Autonomous")
public class Platform extends Auto {

    public void runOpMode() {
        initialize();

        waitForStart();

        try {
            //(x0, y0), (x1, y1), ..., (x3, y3)
            double xcoords[] = {0.0, 10, 0.0, 10};
            double ycoords[] = {0.0, 0.0, 60, 60};

            //strafe(0, 0, "straight", 48);
            //splineMove(xcoords, ycoords, -0.4); //backwards spline from cube to bridge, ending in same orientation
            strafe(.5, 0, "straferight", 200);
        }
        catch (InterruptedException e) { }
    }
}
