package org.firstinspires.ftc.teamcode.Autonomous.Blue;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Autonomous.Auto;

@Autonomous(name = "BluePlatform", group = "Autonomous")
public class BluePlatform extends Auto {

    private ALLIANCE_COLOR color = ALLIANCE_COLOR.BLUE;

    public void runOpMode() {
        initialize();

        waitForStart();

        try {
            gripBlock();
            pause(1.25);

            moveSlideByTicks();

            move(0, .3, 51, "straight");

            strafe(.3, 0, "strafeleft", 16);

            gripPlatform();

            pause(2);

            movePlatform(-.3, color);

            pause(.5);

            strafe(.3, (int)(currentAngle()), "straferight", 16);

            releasePlatform();

            move(currentAngle(), .3, 22, "straight");

            strafe(.3, (int)(currentAngle()), "strafeleft", 16);

            moveByTime(2, (int)(currentAngle()), -.3);

            move(-90, .3, 6, "straight");

            strafe(.3, (int)(currentAngle()), "straferight", 20);

            strafe(.3, (int)(currentAngle()), "strafeleft", 2);

            move(-90, .5, 44, "straight");

            /*strafe(.3, 0, "straferight", 6);

            pause(.5);

            move(0, -.5, 15, "straight");

            pause(.5);

            while(Math.abs(currentAngle()) < 70) {
                turn("cw", .5);
            }
            halt();

            gripBlock();

            pause(1);

            moveSlideByTicks();

            move(-90, -.3, 20, "straight");

            moveByTime(3, -90, -.2);

            move(-90, .3, 2, "straight");

            strafe(.15, -90, "strafeleft", 16);

            gripPlatform();

            pause(2);

            strafe(.25, -90, "straferight", 68);

            pause(.5);

            releasePlatform();

            move(-90, .5, 56, "straight");


            /*strafe(.15, 0, "strafeleft", 6);

            gripBlock();
            pause(1.25);

            strafe(.2, 0, "strafeleft", 18);

            pause(.25);

            move(0, -.3, 11, "straight");

            strafe(.125, 0, "strafeleft", 12);

            pause(.25);

            gripPlatform();

            pause(1);

            strafe(.3, 0, "straferight", 68);

            releasePlatform();

            moveSlideByTicks();

            while (runtime.time() - current < 23) {
                heartbeat();
            }

            move(0, .3, 54, "straight");*/
        }
        catch (InterruptedException e) { }
    }
}
