package org.firstinspires.ftc.teamcode.Autonomous.Red;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Autonomous.Auto;

@Autonomous(name = "RedCollaborativePlatform", group = "Autonomous")
public class RedCollaborativePlatform extends Auto {

    public void runOpMode() {
        initialize();

        waitForStart();

        try {
            leftAutoBlockGrabber.setPosition(0);
            rightAutoBlockGrabber.setPosition(1);
            double initialTime = runtime.time();

            move(0, 25, 180, .6, 0.5, 0.2, "normal", true);
            move(0, 7, 94, .35, .35, .35, "strafe", true);
            move(0, 3.25, 180, .3, 0.3, 0.3, "normal", true);
            gripPlatform();
            pause(0.75);

            move(0, 20, 10, 0.8, 0.5, 0.4, "normal", true);
            moveByTime(1, 0.4, 0);

            releasePlatform();

            while (runtime.time() - initialTime < 25) {
                heartbeat();
            }

            move(0, 24, -84, 0.5, 0.5, 0.5, "strafe", true);
            PIDTurn(-90, 1);
            move(-90, 12, -90, 0.5, 0.5, 0.5, "normal", true);
        }
        catch (InterruptedException e) { }
    }
}
