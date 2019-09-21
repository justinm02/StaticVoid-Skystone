package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Auto", group = "Autonomous")
public class AutoOp extends Auto {

    public void runOpMode() {
        initialize();

        waitForStart();

        try {
            //(x0, y0), (x1, y1), ..., (x3, y3)
            double xcoords[] = {0.0, -10, 0.0, -10};
            double ycoords[] = {0.0, 0.0, -60, -60};

            move(0, .2, "straight");
            //splineMove(xcoords, ycoords, -0.7); //backwards spline from cube to bridge, ending in same orientation
        }
        catch (InterruptedException e) { }
    }
}