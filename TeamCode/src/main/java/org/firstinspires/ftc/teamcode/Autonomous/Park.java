package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Park", group = "Autonomous")
public class Park extends Auto {

    public void runOpMode() {
        initialize();
        waitForStart();

        try {
            move(0, 4, 0, .3,.3,0, "straight", true);
        } catch (InterruptedException e) {
        }
    }
}
