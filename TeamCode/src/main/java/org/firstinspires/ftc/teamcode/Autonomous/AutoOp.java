package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Auto", group = "Autonomous")
public class AutoOp extends Auto {
    private double[] xCoords = new double[]{0, 48, 48};
    private double[] yCoords = new double[]{0,0,48};

    public void runOpMode() {
        initialize();
        waitForStart();
        try {
            //move(0, 96, 0, .8, 0.25, 0, "straight", true);
            moveByTime(15, .5, 0);
            //splineMove(xCoords, yCoords, .75, .25, 0, 0, true);
        }
        catch (InterruptedException e) { }
    }
}