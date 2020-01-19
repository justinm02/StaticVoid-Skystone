package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Auto", group = "Park")
public class AutoOp extends Auto {

    public void runOpMode() {
        initialize();
        waitForStart();

        try {
            move(0, 120, 180, .5, 0, 0, "straight");
        //PIDTurn(150);
        //strafe(.25, 0, "strafeleft", 48);
        //move(0, -.3, 200, "straight");
        //moveSlideByTicks(400, .5);
            // diagonalStrafe(-45, -90, 0, 0);
            //determineSkystonePosition();
            //move(0, 120, -90, .5, "strafe");
            //move(0, 120, 90, 1);
        }
        catch (InterruptedException e) { }
    }
}