package org.firstinspires.ftc.teamcode.Autonomous.Red;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.Autonomous.Auto;


@Autonomous(name = "RedSkystone", group = "Autonomous")
public class RedSkystone extends Auto {

    private double xSkystone[], ySkystone[];
    private double xCrossBridge[], yCrossBridge[];
    private double xSkystone2[], ySkystone2[];

    private ALLIANCE_COLOR color = ALLIANCE_COLOR.RED;

    public void runOpMode() {
        initialize();
        waitForStart();

        //start auto
        try {
            moveFromWall();

            SKYSTONE_POSITION position = predetermineMovement();

            getFirstSkystone();

            deliverFirstSkystone();

            getAndDeliverSecondSkystone(position);

            parkBot();
        }
        catch(InterruptedException e) { }
    }

    public void moveFromWall() throws InterruptedException {
        strafe(.15, 0, "straferight", 6);

        adjustClaw();
        pause(1.25);
    }

    public SKYSTONE_POSITION predetermineMovement() throws InterruptedException {
        SKYSTONE_POSITION position = determineSkystonePlacement(color);

        if (position.equals(SKYSTONE_POSITION.RIGHT)) {
            xSkystone = new double[]{0, 14, 14};
            ySkystone = new double[]{0, 0, -8};

            xCrossBridge = new double[]{0, 10, 10, 10};
            yCrossBridge = new double[]{0, -15, -47, -65};

            xSkystone2 = new double[]{0, -44, -44};
            ySkystone2 = new double[]{0, 0, -12};

        } else if (position.equals(SKYSTONE_POSITION.MIDDLE)) {
            xSkystone = new double[]{0, 24, 24};
            ySkystone = new double[]{0, 0, -6};

            xCrossBridge = new double[]{0, 10, 10, 10};
            yCrossBridge = new double[]{0, -15, -47, -73};

            xSkystone2 = new double[]{0, -56, -56};
            ySkystone2 = new double[]{0, 0, -12};
        } else {
            strafe(.3, 0, "strafeleft", 3);
            move(0, -.35, 3, "straight");

            xSkystone = new double[]{0, 10, 10};
            ySkystone = new double[]{0, 0, -8};

            xCrossBridge = new double[]{0, 10, 10, 10};
            yCrossBridge = new double[]{0, -15, -47, -55};

            xSkystone2 = new double[]{0, -56, -56};
            ySkystone2 = new double[]{0, 0, -12};
        }
        return position;
    }

    public void getFirstSkystone() throws InterruptedException {
        splineMove(xSkystone, ySkystone, .3, 0); //backwards spline from cube to bridge, ending in same orientation

        move(-90, .4, 22, "straight");

        gripBlock();
        pause(.5);
    }

    public void deliverFirstSkystone() throws InterruptedException {
        move(-90, -.4, 42, "straight");

        splineMove(xCrossBridge, yCrossBridge, .8, -Math.PI / 2);
        //strafe(.25, 179, "straferight", 2);
        adjustClaw();
    }

    public void getAndDeliverSecondSkystone(SKYSTONE_POSITION position) throws InterruptedException {
        if (position.equals(SKYSTONE_POSITION.MIDDLE)) {
            move(179, -.8, 34, "straight");
            pause(.5);
            moveByTime(2.75, 179, -.3);

            move(179, .2, 9, "straight");

            strafe(.25, 179, "strafeleft", 16);

            move(179, .25, 10,"straight");

            gripBlock();

            pause(.5);

            strafe(.5, 179, "straferight", 20);

            move(179, .7, 63, "straight");
        }

        else if (position.equals(SKYSTONE_POSITION.LEFT)) {
            move(179, -.8, 54, "straight");
            pause(.5);

            //turn(0, .5, "left");

            while (Math.abs(currentAngle()) > 35) {
                turn("cw", .85);
            }

            halt();

            strafe(.25, 0, "straferight", 18);

            moveByTime(.75, 0, .45);

            gripBlock();

            pause(.5);

            move(0, -.3, 3, "straight");

            strafe(.5, 0, "strafeleft", 23);

            while (Math.abs(currentAngle()) < 130) {
                turn("cw", .85);
            }

            move(179, .9, 70, "straight");
        }

        else {
            move(180, -.8, 46, "straight");
            pause(.5);
            moveByTime(2, 180, -.3);

            pause(.25);
            move(180, .4, 15,"straight");

            strafe(.25, 180, "strafeleft", 18);

            move(180, .4, 7,"straight");

            gripBlock();

            pause(1);

            strafe(.5, 180, "straferight", 24);

            move(180, .7, 45, "straight");
        }
        pause(.5);
    }

    public void parkBot() throws InterruptedException {
        adjustClaw();

        move(179, -.7, 12, "straight");
    }
}
