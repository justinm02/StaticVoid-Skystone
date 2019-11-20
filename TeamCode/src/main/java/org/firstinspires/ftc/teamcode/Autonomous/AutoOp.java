package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Auto", group = "Autonomous")
public class AutoOp extends Auto {

    public void runOpMode() {
        initialize();

        waitForStart();

        try {
            //strafe(.15, 0, "straferight", 200);
            move(0, 0, 200, "straight");
        }
        catch (InterruptedException e) { }
    }
}