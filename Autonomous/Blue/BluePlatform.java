package org.firstinspires.ftc.teamcode.Autonomous.Blue;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Autonomous.Auto;

@Autonomous(name = "BluePlatform", group = "Autonomous")
public class BluePlatform extends Auto {

    private ALLIANCE_COLOR color = ALLIANCE_COLOR.BLUE;
    private double xPlatform[] = new double[]{0, -26, -26};
    private double yPlatform[] = new double[]{0, 0, -12};

    public void runOpMode() {
        initialize();

        waitForStart();

        try {
            //move up and grab
            adjustClaw();
            move(0, .65, 20, "straight");
            moveSlideByTicks(400, .65);
            pause(.5);
            strafe(.4, 0, "strafeleft", 8);
            move(0, .2, 6, "straight");
            pause(.5);
            moveSlideByTicks(400, -.3);
            pause(.5);

            //pull & push into wall
            splineMove(xPlatform, yPlatform, -.3, 0);
            moveByTime(.75, 90, .4);
            moveSlideByTicks(300, .65);
            pause(.5);

            //park
            move(90, -.65, 4, "straight");
            strafe(.4, 90, "strafeleft", 4);
            moveSlideByTicks(200, -.3);
            plow();
            move(90, -.65, -30, "straight");
        }
        catch (InterruptedException e) { }
    }
}