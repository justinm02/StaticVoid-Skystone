package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Autonomous.Spline.Waypoint;

@Autonomous(name = "StrafePIDTest", group = "Autonomous")
public class Strafe extends Auto {

    public void runOpMode() {
        initialize();
        waitForStart();

        try {

            newMove(new Waypoint(0,0), new Waypoint(0, 120), 60, .3, .7, .3);
        } catch (InterruptedException e) {
        }
    }
}
