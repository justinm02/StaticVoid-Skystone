package org.firstinspires.ftc.teamcode.Autonomous.Red;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.Autonomous.Auto;


@Autonomous(name = "RedSkystone", group = "Autonomous")
public class Skystone extends Auto {

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
            xSkystone = new double[]{0, 12, 12};
            ySkystone = new double[]{0, 0, -8};

            xCrossBridge = new double[]{0, 10, 1, 0};
            yCrossBridge = new double[]{0, -15, -47, -65};

            xSkystone2 = new double[]{0, -44, -44};
            ySkystone2 = new double[]{0, 0, -12};

        } else if (position.equals(SKYSTONE_POSITION.MIDDLE)) {
            xSkystone = new double[]{0, 21.5, 21.5};
            ySkystone = new double[]{0, 0, -6};

            xCrossBridge = new double[]{0, 10, 3, -3};
            yCrossBridge = new double[]{0, -15, -47, -70};

            xSkystone2 = new double[]{0, -56, -56};
            ySkystone2 = new double[]{0, 0, -12};
        } else {
            strafe(.3, 0, "strafeleft", 3);
            move(0, -.35, 3, "straight");

            xSkystone = new double[]{0, 9, 9};
            ySkystone = new double[]{0, 0, -8};

            xCrossBridge = new double[]{0, 10, 3, 1};
            yCrossBridge = new double[]{0, -15, -47, -55};

            xSkystone2 = new double[]{0, -56, -56};
            ySkystone2 = new double[]{0, 0, -12};
        }
        return position;
    }

    public void getFirstSkystone() throws InterruptedException {
        splineMove(xSkystone, ySkystone, .3, 0); //backwards spline from cube to bridge, ending in same orientation

        move(-90, .4, 16, "straight");

        gripBlock();
        pause(.5);
    }

    public void deliverFirstSkystone() throws InterruptedException {
        move(-90, -.4, 38, "straight");

        splineMove(xCrossBridge, yCrossBridge, .8, -Math.PI / 2);
        adjustClaw();
    }

    public void getAndDeliverSecondSkystone(SKYSTONE_POSITION position) throws InterruptedException {
        if (position.equals(SKYSTONE_POSITION.MIDDLE)) {
            move(-180, -.8, 40, "straight");
            pause(.5);
            moveByTime(2, -180, -.3);

            move(-180, .4, 6, "straight");

            strafe(.5, -180, "strafeleft", 20);

            move(-180, .25, 10,"straight");

            gripBlock();

            pause(.5);

            strafe(.5, -180, "straferight", 16);

            move(-180, .8, 50, "straight");
        }

        else if (position.equals(SKYSTONE_POSITION.LEFT)) {
            move(-180, -.8, 44, "straight");
            pause(.5);
            moveByTime(1, -180, -.3);

            move(-180, .5, 6, "straight");

            pause(.5);

            //turn(0, .5, "left");

            while (Math.abs(currentAngle()) > 25) {
                turn("cw", .7);
            }

            halt();

            pause(.25);

            strafe(.5, 0, "straferight", 22);

            moveByTime(.75, 0, .45);

            gripBlock();

            pause(.5);

            move(0, -.3, 3, "straight");

            strafe(.5, 0, "strafeleft", 20);

            move(0, -.9, 53, "straight");


            while (Math.abs(currentAngle()) < 130) {
                turn("cw", .8);
            }
        }

        else {
            move(-180, -.8, 40, "straight");
            pause(.5);
            moveByTime(2, -180, -.3);

            pause(.25);
            move(-180, .4, 14,"straight");

            strafe(.5, -180, "strafeleft", 18);

            move(-180, .4, 11,"straight");

            gripBlock();

            pause(1);

            strafe(.5, -180, "straferight", 16);

            move(-180, .8, 35, "straight");
        }
    }

    public void parkBot() throws InterruptedException {
        adjustClaw();

        move(-180, -.7, 7, "straight");
    }
}
