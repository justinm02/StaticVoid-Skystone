package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "StrafePIDTest", group = "Autonomous")
public class Strafe extends Auto {

    public void runOpMode() {
        initialize();
        waitForStart();

        try {

            move(0, 120, 90, .5,.3,0, "strafe", true);
        } catch (InterruptedException e) {
        }
    }
}
