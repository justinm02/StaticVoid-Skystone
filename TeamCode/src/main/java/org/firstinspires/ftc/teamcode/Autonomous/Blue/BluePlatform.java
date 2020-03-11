package org.firstinspires.ftc.teamcode.Autonomous.Blue;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Autonomous.Auto;

@Autonomous(name = "BluePlatform", group = "Autonomous")
public class BluePlatform extends Auto {

    public void runOpMode() {
        initialize();

        waitForStart();

        try {
            moveByWheelEncoders(0, 30, -.5, "straight");
            strafeByWheelEncoders(0, 8, .3, "straferight");
            moveByWheelEncoders(0, 2, -.3, "straight");

            gripPlatform();
            pause(2);
            moveByWheelEncoders(0, 20, .5, "straight");
            PIDTurn(90, .5);
            releasePlatform();
            pause(2);
            moveByTime(2 ,-.3, 90);
            strafeByWheelEncoders(90, 36, .5, "straferight");
            strafeByWheelEncoders(-90, 3, .3, "straferight");
            grip();
            moveByWheelEncoders(90, 40, .5, "straight");
        }
        catch (InterruptedException e) { }
    }
}