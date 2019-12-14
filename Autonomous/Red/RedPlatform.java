package org.firstinspires.ftc.teamcode.Autonomous.Red;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Autonomous.Auto;

@Autonomous(name = "RedPlatform", group = "Autonomous")
public class RedPlatform extends Auto {

    private ALLIANCE_COLOR color = ALLIANCE_COLOR.RED;
    private double xPlatform[] = new double[]{0, -26, -26};
    private double yPlatform[] = new double[]{0, 0, 12};

    public void runOpMode() {
        initialize();

        waitForStart();

        try {
            //move up and grab
            adjustClaw();
            move(0, .65, 20, "straight");
            moveSlideByTicks(400, .65);
            pause(.5);
            strafe(.4, 0, "straferight", 8);
            move(0, .2, 6, "straight");
            pause(.5);
            moveSlideByTicks(400, -.3);
            pause(.5);

            //pull & push into wall
            splineMove(xPlatform, yPlatform, -.3, 0); //CORRECTION DOESN'T WORK FIX LATER
            /*move(0, -.4, 10, "straight");
            PIDTurn(-15, 1.0);
            move(-25, -.4, 15, "straight");
            PIDTurn(-90, 1.0);*/
            moveByTime(1, -90, .4);
            moveSlideByTicks(300, .65);
            pause(.5);

            //park
            move(-90, -.65, 4, "straight");
            strafe(.4, -90, "straferight", 4);
            plow();
            move(-90, -.65, -30, "straight");
        }
        catch (InterruptedException e) { }
    }
}