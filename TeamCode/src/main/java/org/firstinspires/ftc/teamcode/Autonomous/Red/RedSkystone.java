package org.firstinspires.ftc.teamcode.Autonomous.Red;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Autonomous.Auto;


@Autonomous(name = "RedSkystone", group = "Autonomous")
public class RedSkystone extends Auto {

    private double xSkystone[], ySkystone[], xSkystone2[], ySkystone2[], movePlatformX[];
    private double xPlatform[], yPlatform[], xCrossBridge[], yCrossBridge[], movePlatformY[];

    private SKYSTONE_POSITION position;

    public void runOpMode() {
        initialize();
        waitForStart();

        try {
            position = determineSkystonePosition("red");

//            double current = runtime.time();
//            while (runtime.time() - current < 2) {
//                telemetry.addData("position", position);
//                telemetry.update();
//            }

            bringAlignerDown();


            movePlatformX = new double[]{0, 20, 20};
            movePlatformY = new double[]{0, 0, -14};

            if (position.equals(SKYSTONE_POSITION.MIDDLE)) {
                goMiddle();
            }
            else if (position.equals(SKYSTONE_POSITION.LEFT)) {
                goLeft();
            }
            else {
                goRight();
            }

        } catch (InterruptedException e) {
        }
    }

    public void goLeft() throws InterruptedException {
        move(0, 28, -75, .8, .4, 0, "strafe", true);
        xSkystone = new double[]{0, -5, -8, -30};
        ySkystone = new double[]{0, 0, 5, 5};

        xPlatform = new double[]{0, -15, -15, -48};
        yPlatform = new double[]{5, 5, -2, -2};

        xCrossBridge = new double[]{-43, -15, -15, 0};
        yCrossBridge = new double[]{-3, -3, 2, 2};

        xSkystone2 = new double[]{0, -5, -8, -46};
        ySkystone2 = new double[]{0, 0, 3, 3};

        grabBlock();

        move(0, 3, 90, .3, .35, 0, "strafe", true);

        splineMove(xSkystone, ySkystone, -.6, -.25, -.35, 0, false);
        splineMove(xPlatform, yPlatform, -.35, -.35, 0, 0, true);
        move(0, 1, -90, .3, .3, .3, "strafe", true);

        placeStone();

        move(0, 1, 90, .3, .3, 0, "strafe", true);

        splineMove(xCrossBridge, yCrossBridge, .8, .35, .65, 0, false);
        bringAlignerDown();

        move(0, 50, -23, .65, .5, .5, "straight", false);
        moveByTime(0.5, 0.5, 0);

        move(0, 5, -91, .5, .5, .3, "strafe", true);

        grabBlock();
        pause(.25);

        move(0, 2, 90, .5, .5, .3, "strafe", true);
        //move(0, 60, 152, .8, .5, .5, "strafe", false);

        //move(0, 35, -145, .5, .5, .5, "strafe", true);

        splineMove(xSkystone2, ySkystone2, -.75, -.35, -.6, 0, false);
        splineMove(xPlatform, yPlatform, -.6, -.6, -.45, 0, false);
        move(0, 17, 150, .45, .45, .45, "straight", true);

        placeStone();
        pause(.25);

        getPlatform();

        park();
    }

    public void goMiddle() throws InterruptedException {
        move(0, 27, -88, .7, .4, 0, "strafe", true);
        xSkystone = new double[]{0, -5, -8, -30};
        ySkystone = new double[]{0, 0, 5, 5};

        xPlatform = new double[]{0, -15, -15, -40};
        yPlatform = new double[]{5, 5, -3, -3};

        xCrossBridge = new double[]{-35, -15, -15, 0};
        yCrossBridge = new double[]{-3, -3, 3, 3};

        xSkystone2 = new double[]{0, -5, -8, -44};
        ySkystone2 = new double[]{0, 0, 3, 3};

        grabBlock();

        move(0, 1, 90, .3, .35, 0, "strafe", true);

        splineMove(xSkystone, ySkystone, -.6, -.35, -.35, 0, false);
        splineMove(xPlatform, yPlatform, -.35, -.35, 0, 0, true);
        move(0, 1, -90, .3, .3, .3, "strafe", true);

        placeStone();

        move(0, 1, 90, .3, .3, 0, "strafe", true);

        splineMove(xCrossBridge, yCrossBridge, .8, .35, .65, 0, false);
        bringAlignerDown();
        if (position != SKYSTONE_POSITION.LEFT) {
            move(0, 58, -25, .65, .5, .5, "strafe", true);
        }
        else {
            move(0, 50, -25, .65, .5, .5, "straight", false);
            moveByTime(1.25, 0.5, 0);
        }

        move(0, 2, -87, .3, .3, 0, "strafe", true);

        grabBlock();
        pause(.25);

        move(0, 2, 90, .3, .3, 0, "strafe", true);
        //move(0, 60, 152, .8, .5, .5, "strafe", false);

        //move(0, 35, -145, .5, .5, .5, "strafe", true);

        splineMove(xSkystone2, ySkystone2, -.6, -.35, -.6, 0, false);
        splineMove(xPlatform, yPlatform, -.6, -.6, -.45, 0, false);
        move(0, 17, 163, .45, .45, .3, "straight", true);

        placeStone();
        pause(.25);

        getPlatform();

        park();
    }

    public void goRight() throws InterruptedException {
        move(0, 26, -100, .7, .4, 0, "strafe", true);
        xSkystone = new double[]{0, -5, 0, -25};
        ySkystone = new double[]{0, 2, 2, 2};

        xPlatform = new double[]{0, -15, -15, -40};
        yPlatform = new double[]{5, 5, -2, -2};

        xCrossBridge = new double[]{-27, -15, -15, 5};
        yCrossBridge = new double[]{-3, -3, 1, 1};

        xSkystone2 = new double[]{0, -5, 0, -44};
        ySkystone2 = new double[]{0, 2, 0, 0};

        grabBlock();

        move(0, 1, 90, .3, .35, 0, "strafe", true);

        splineMove(xSkystone, ySkystone, -.6, -.25, -.35, 0, false);
        splineMove(xPlatform, yPlatform, -.35, -.35, 0, 0, true);
        move(0, 3, -90, .3, .3, .3, "strafe", true);

        placeStone();

        move(0, 1, 90, .3, .3, 0, "strafe", true);

        splineMove(xCrossBridge, yCrossBridge, .8, .25, .65, 0, false);
        bringAlignerDown();

        move(0, 50, -5, .65, .5, .5, "strafe", true);

        move(0, 5, -87, .3, .3, .2, "strafe", true);

        grabBlock();
        pause(.25);

        move(0, 2, 90, .3, .3, 0, "strafe", true);
        //move(0, 60, 152, .8, .5, .5, "strafe", false);

        //move(0, 35, -145, .5, .5, .5, "strafe", true);

        splineMove(xSkystone2, ySkystone2, -.6, -.3, -.6, 0, false);
        splineMove(xPlatform, yPlatform, -.6, -.6, -.45, 0, false);
        move(0, 17, 168, .45, .45, .3, "straight", true);

        placeStone();
        pause(.25);

        getPlatform();

        park();
    }

    public void grabBlock() throws InterruptedException {
        gripBlock();
        pause(.5);
        bringAlignerUp();
    }

    public void placeStone() throws InterruptedException {
        bringAlignerDown();
        pause(.25);
        releaseBlock();
        pause(.1);
        bringAlignerUp();
    }

    public void getPlatform() throws InterruptedException {
        PIDTurn(90, .75);
        move(90, 3, -90, .3, .3, .3, "straight", true);

        gripPlatform();
        pause(.5);

        splineMove(movePlatformX, movePlatformY, .5, .3, 0, Math.PI/2, true);

        releasePlatform();
        pause(.5);
    }

    public void park() throws InterruptedException {
        move(0, 10, -77, 0.85, 0.4, 0.3, "strafe", true);
        move(0, 26, 0, 0.85, .4, .4, "straight", true);
    }
}