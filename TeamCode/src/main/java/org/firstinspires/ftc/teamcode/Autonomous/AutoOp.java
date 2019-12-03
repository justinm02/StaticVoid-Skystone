package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Auto", group = "Autonomous")
public class AutoOp extends Auto {

    public void runOpMode() {
        initialize();
        waitForStart();

        //try {
            //PIDTurn(150);
            //strafe(.25, 0, "strafeleft", 48);
            //move(0, -.3, 200, "straight");
            moveSlideByTicks(400, .5);
        //}
        //catch (InterruptedException e) { }
    }
}