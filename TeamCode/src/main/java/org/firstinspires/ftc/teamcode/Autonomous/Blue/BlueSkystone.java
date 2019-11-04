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

            getFirstSkystone();

            deliverFirstSkystone();

            getAndDeliverSecondSkystone(position);

            parkBot();
        }
        catch (InterruptedException e) { }
    }

    public void moveFromWall() throws InterruptedException {
        strafe(.2, 0, "straferight",6);

        adjustClaw();
        pause(1.25);
    }

    public SKYSTONE_POSITION predetermineMovement() throws InterruptedException {
        xSkystone = new double[]{0, 9, 9};
        ySkystone = new double[]{0, 0, -6};

        SKYSTONE_POSITION position = determineSkystonePlacement(color);

        if (position.equals(SKYSTONE_POSITION.RIGHT)) {
            moveByTime(2, 0, -.3);

            xCrossBridge = new double[]{0, 7.5, 0, 3};
            yCrossBridge = new double[]{0, 15, 15, 35};
        }

        else if (position.equals(SKYSTONE_POSITION.MIDDLE)) {
            move(0, -.3, 13, "straight");

            xCrossBridge = new double[]{0, 7.5, 0, 3};
            yCrossBridge = new double[]{0, 15, 15, 30};
        }

        else {
            move(0, -.3, 3, "straight");

            xCrossBridge = new double[]{0, 5, 0, 3};
            yCrossBridge = new double[]{0, 5, 10, 20};
        }

        return position;
    }

    public void getFirstSkystone() throws InterruptedException {
        splineMove(xSkystone, ySkystone, .3, 0);

        move(-90, .45, 18, "straight");

        gripBlock();

        pause(1);
    }

    public void deliverFirstSkystone() throws InterruptedException {
        move(-90, -.3, 37, "straight");

        splineMove(xCrossBridge, yCrossBridge, .7, -Math.PI/2);

        adjustClaw();
    }

    public void getAndDeliverSecondSkystone(SKYSTONE_POSITION position) throws InterruptedException {
        if (position.equals(SKYSTONE_POSITION.LEFT)) {
            move(0, -.5, 43, "straight");

            pause(.25);

            while (Math.abs(currentAngle()) < 140) {
                turn("cw", .7);
            }

            strafe(.5, -180, "strafeleft", 22);

            moveByTime(.75, -180, .45);

            gripBlock();

            strafe(.5, -180, "straferight", 22);

            move(-180, -.8, 48, "straight");
        }

        else if (position.equals(SKYSTONE_POSITION.MIDDLE)) {
            move(0, -.5, 38, "straight");

            pause(.25);

            while (Math.abs(currentAngle()) < 140) {
                turn("cw", .7);
            }

            strafe(.5, -180, "strafeleft", 34);

            moveByTime(.75, -180, .45);

            gripBlock();

            strafe(.5, -180, "straferight", 24);

            move(-180, -.8, 56, "straight");
        }

        else {
            move(0, -.5, 56, "straight");

            pause(.25);

            while (Math.abs(currentAngle()) < 140) {
                turn("cw", .7);
            }

            strafe(.5, -180, "strafeleft", 24);

            moveByTime(.75, -180, .45);

            gripBlock();

            strafe(.5, -180, "straferight", 22);

            move(-180, -.8, 64, "straight");
        }
    }

    public void parkBot() throws InterruptedException {
        while (Math.abs(currentAngle()) > 25) {
            turn("ccw", .7);
        }

        adjustClaw();

        move(0, -.8, 7, "straight");
    }
}

