package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Auto", group = "Autonomous")
public class AutoOp extends Auto {

    public void runOpMode() {
        initialize();
        waitForStart();

        try {
            //PIDTurn(90);
            //strafe(.25, 0, "strafeleft", 48);
            move(0, -.3, 200, "straight");
        }
        catch (InterruptedException e) { }
    }
}