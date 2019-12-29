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
        //move(0, -
        gripBlock();
        moveSlideByTicks(600, .65);
        //}
        //catch (InterruptedException e) { }
    }
}