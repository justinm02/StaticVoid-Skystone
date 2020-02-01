package org.firstinspires.ftc.teamcode.Autonomous.Red;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Autonomous.Auto;

@Autonomous(name = "RedPlatform", group = "Autonomous")
public class RedPlatform extends Auto {

    public void runOpMode() {
        initialize();

        waitForStart();

        try {
            moveByWheelEncoders(0, 30, -.5, "straight");
            strafeByWheelEncoders(0, 6, .5, "strafeleft");
            moveByWheelEncoders(0, 2, -.3, "straight");
            gripPlatform();
            pause(2);
            moveByWheelEncoders(0, 10, .5, "straight");
            PIDTurn(-90, .5);
            releasePlatform();
            pause(.5);
            moveByTime(2, -.3, -90);
            strafeByWheelEncoders(-90, 34, .3, "strafeleft");
            moveByWheelEncoders(-90, 40, .5, "straight");
        }
        catch (InterruptedException e) { }
    }
}
