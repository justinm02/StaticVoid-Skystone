package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Autonomous.Spline.Waypoint;

@Autonomous(name = "StrafePIDTest", group = "Autonomous")
public class Strafe extends Auto {

    public void runOpMode() {
        initialize();
        waitForStart();

        try {
            bringAlignerDown();
            newMove(new Waypoint(0,0), new Waypoint(-15, -27), 0, .2, .3, 0, true);
        } catch (InterruptedException e) {
        }
    }
}
