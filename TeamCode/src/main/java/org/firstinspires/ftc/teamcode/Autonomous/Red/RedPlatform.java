package org.firstinspires.ftc.teamcode.Autonomous.Red;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Autonomous.Auto;

@Autonomous(name = "RedPlatform", group = "Autonomous")
public class RedPlatform extends Auto {

    private ALLIANCE_COLOR color = ALLIANCE_COLOR.RED;

    public void runOpMode() {
        initialize();

        waitForStart();

        try {
            move(0, -.35, 44, "straight");

            strafe(.3, 0, "strafeleft", 16);

            move(0, -.3, 2, "straight");

            gripPlatform();

            pause(1.25);

            movePlatform(.3, color);

            pause(.25);

            while (Math.abs(currentAngle()) < 90) {
                turn("ccw", .2);
            }

            strafe(.4, 90, "straferight", 12);

            releasePlatform();

            pause(1.25);

            move(90, -.6, 33, "straight");

            strafe(.25, 90, "strafeleft", 26);

            moveByTime(2, 90, .3);

            move(90, -.8, 10, "straight");

            gripBlock();
            pause(1.25);

            //moveSlideByTicks();

            strafe(.3, (int)(currentAngle()), "straferight", 32);

            strafe(.3, (int)(currentAngle()), "strafeleft", 2);

            move(90, -.8, 25, "straight");
        }
        catch (InterruptedException e) { }
    }
}
