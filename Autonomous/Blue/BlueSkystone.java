package org.firstinspires.ftc.teamcode.Autonomous.Blue;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.Autonomous.Auto;


@Autonomous(name = "BlueSkystone", group = "Autonomous")
public class BlueSkystone extends Auto {

    private double xSkystone[], ySkystone[];
    private double xCrossBridge[], yCrossBridge[];
    private double xSkystone2[], ySkystone2[];

    private ALLIANCE_COLOR color = ALLIANCE_COLOR.BLUE;

    public void runOpMode() {
        initialize();
        waitForStart();

        //start auto
        try {
            moveFromWall();

            SKYSTONE_POSITION position = predetermineMovement();

            getFirstSkystone(position);

            deliverFirstSkystone(position);

            //getAndDeliverSecondSkystone(position);

            parkBot();
        }
        catch (InterruptedException e) { }
    }

    public void moveFromWall() throws InterruptedException {
        strafe(.35, 0, "straferight",6);

        adjustClaw();
        pause(0.75);
    }

    public SKYSTONE_POSITION predetermineMovement() throws InterruptedException {
        xSkystone = new double[]{0, 11, 11};
        ySkystone = new double[]{0, 0, -6};

        SKYSTONE_POSITION position = determineSkystonePlacement(color);

        if (position.equals(SKYSTONE_POSITION.RIGHT)) {
            xSkystone = new double[]{0, 14.5, 14.5};
            ySkystone = new double[]{0, 0, -6};
            moveByTime(2, 0, -.3);

            xCrossBridge = new double[]{0, 5, 5};
            yCrossBridge = new double[]{0, 0, 5};
        }

        else if (position.equals(SKYSTONE_POSITION.MIDDLE)) {
            move(0, -.3, 8, "straight");

            xCrossBridge = new double[]{0, 5, -1, 0};
            yCrossBridge = new double[]{0, 5, 15, 28};
        }

        else {
            strafe(.2, 0, "strafeleft",2);
            xCrossBridge = new double[]{0, 5, -1, 0};
            yCrossBridge = new double[]{0, 5, 15, 22};
        }

        return position;
    }

    public void getFirstSkystone(SKYSTONE_POSITION position) throws InterruptedException {
        splineMove(xSkystone, ySkystone, .3, 0);

        move(-90, .5, 23, "straight");

        gripBlock();

        pause(1);

        if (position.equals(SKYSTONE_POSITION.LEFT)) {
            move(-90, -.3, 14, "straight");
            strafe(.4, -90, "straferight", 6);
        }
    }

    public void deliverFirstSkystone(SKYSTONE_POSITION position) throws InterruptedException {
        if (position.equals(SKYSTONE_POSITION.LEFT)) {
            move(-90, -.3, 25, "straight");
        }
        else
        {
            move(-90, -.3, 39, "straight");
        }

        //splineMove(xCrossBridge, yCrossBridge, .8, -Math.PI/2);

        move(-90, .3, 24, "straight");
        while(currentAngle() < 0) {
            turn("ccw", .2);
        }

        if (position.equals(SKYSTONE_POSITION.LEFT)) {
            move(0, .3, 49, "straight");

            /*while (Math.abs(currentAngle()) > 15) {
                turn("ccw", .3);
            }
            strafe(.35, 0, "straferight", 6);
            move(0, .5, 44, "straight");*/
        }

        else if (position.equals(SKYSTONE_POSITION.MIDDLE)) {
            move(0, .3, 57, "straight");

            /*while (Math.abs(currentAngle()) > 15) {
                turn("ccw", .3);
            }
            strafe(.35, 0, "straferight", 4);
            move(0, .5, 45, "straight");*/
        }
        else if (position.equals(SKYSTONE_POSITION.RIGHT)) {
            move(0, .3, 65, "straight");

            /*move(0, .5, 62, "straight");
            strafe(.35, 0, "strafeleft", 2);*/
        }
        halt();
    }

    public void getAndDeliverSecondSkystone(SKYSTONE_POSITION position) throws InterruptedException {
        if (position.equals(SKYSTONE_POSITION.LEFT)) {
            move(0, -.75, 40, "straight");

            pause(.25);

            while (Math.abs(currentAngle()) < 140) {
                turn("cw", .85);
            }
            halt();

            move(179, .25, 3, "straight");

            strafe(.35, 179, "strafeleft", 25);

            moveByTime(.75, 179, .45);

            gripBlock();

            strafe(.35, 179, "straferight", 22);

            while (Math.abs(currentAngle()) > 40) {
                turn("ccw", .85);
            }
            halt();

            move(0, .9, 51, "straight");
        }

        else if (position.equals(SKYSTONE_POSITION.MIDDLE)) {
            move(0, -.5, 39, "straight");

            pause(.25);

            while (Math.abs(currentAngle()) < 140) {
                turn("cw", .7);
            }
            halt();

            pause(.5);

            move(170, .5, 8, "straight");

            strafe(.5, 179, "strafeleft", 22);

            moveByTime(.75, 179, .45);

            gripBlock();

            strafe(.5, 179, "straferight", 22);

            while (Math.abs(currentAngle()) > 40) {
                turn("ccw", .85);
            }
            halt();

            move(0, .8, 62, "straight");
        }

        else {
            move(0, -.5, 56, "straight");

            pause(.25);

            while (Math.abs(currentAngle()) < 140) {
                turn("cw", .85);
            }

            move(179, .5, 4, "straight");

            strafe(.5, 179, "strafeleft", 24);

            moveByTime(.75, 179, .45);

            gripBlock();

            strafe(.5, 179, "straferight", 25);

            while (Math.abs(currentAngle()) > 40) {
                turn("cw", .85);
            }

            move(179, .8, 76, "straight");
        }
    }

    public void parkBot() throws InterruptedException {
        adjustClaw();

        move(0, -.3, 12, "straight");
    }
}
