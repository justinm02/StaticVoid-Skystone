package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Platform", group = "Autonomous")
public class Platform extends Auto {

    public void runOpMode() {
        initialize();

        waitForStart();

        try {
            strafe(.35, 0, "strafeleft", 54);

            pause(.25);

            move(0, .3, 6, "straight");

            gripPlatform();

            pause(1);

            strafe(.3, 0, "straferight", 54);
        }
        catch (InterruptedException e) { }
    }
}
