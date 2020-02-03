package org.firstinspires.ftc.teamcode.Autonomous.Blue;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Autonomous.Auto;


@Autonomous(name = "BlueSkystone", group = "Autonomous")
public class BlueSkystone extends Auto {

    private double xSkystone[], ySkystone[], xSkystone2[], ySkystone2[], movePlatformX[];
    private double xPlatform[], yPlatform[], xCrossBridge[], yCrossBridge[], movePlatformY[];

    private SKYSTONE_POSITION position;

    public void runOpMode() {
        initialize();
        waitForStart();

        try {
            position = determineSkystonePosition("blue");

            double current = runtime.time();
            while (runtime.time() - current < 0.125) {
                telemetry.addData("position", position);
                telemetry.update();
            }

            bringAlignerDown();

            if (position.equals(SKYSTONE_POSITION.MIDDLE)) {
                goMiddle();
            } else if (position.equals(SKYSTONE_POSITION.LEFT)) {
                goLeft();
            } else {
                goRight();
            }
        } catch (InterruptedException e) {

        }
    }

    public void goMiddle() throws InterruptedException {
        xSkystone = new double[]{0, 5, 8, 40};
        ySkystone = new double[]{0, 0, 4, 4};

        xPlatform = new double[]{0, 15, 15, 36};
        yPlatform = new double[]{5, 5, 0, 0};

        xCrossBridge = new double[]{43, 15, 15, 3};
        yCrossBridge = new double[]{-3, -3, 0, 0};

        xSkystone2 = new double[]{0, 5, 8, 45};
        ySkystone2 = new double[]{0, 0, 4, 4};

        movePlatformX = new double[]{0, 20, 20};
        movePlatformY = new double[]{0, 0, 14};

        move(0, 28, -90, .6, .35, 0, "strafe", true);
        grabBlock();
        move(0, 1, 90, .5, .4, .3, "strafe", true);

        splineMove(xSkystone, ySkystone, .65, .35, .35, 0, false);
        splineMove(xPlatform, yPlatform, .35, .35, .2, 0, true);

        move(0, 3, -90, .5, .5, .4, "strafe", true);

        placeStone();

        move(0, 1.5, 90, .5, .5, 0, "strafe", true);
        grip();

        splineMove(xCrossBridge, yCrossBridge, -.8, -.35, -.65, 0, false);
        bringAlignerDown();
        releaseBlock();

        move(0, 48, 180, .65, .65, .5, "straight", true);
        moveByTime(.65, -.5, 0);

        move(0, 6, -90, .5, .5, .5, "strafe", true);

        grabBlock();
        pause(.25);

        move(0, 2, 90, .5, .5, .5, "strafe", true);

        splineMove(xSkystone2, ySkystone2, .75, .35, .6, 0, false);
        splineMove(xPlatform, yPlatform, .6, .6, .6, 0, false);
        move(0, 26, 0, .6, .6, .45, "straight", true);
        pause(0.5);

        move(0, 4, -90, .5, .5, .5, "strafe", true);
        placeStone();

        getPlatform("middle");

        grip();
        move(180, 8.5, -103, 0.85, 0.55, 0.7, "strafe", true);
        move(180, 17, 180, 0.85, 0.55, 0.7, "straight", true);
    }

    public void goLeft() throws InterruptedException {
        xSkystone = new double[]{0, 5, 8, 30};
        ySkystone = new double[]{0, 0, 3, 3};

        xPlatform = new double[]{0, 15, 15, 32};
        yPlatform = new double[]{5, 5, 0, 0};

        xCrossBridge = new double[]{43, 15, 15, 0};
        yCrossBridge = new double[]{-3, -3, 1.5, 1.5};

        xSkystone2 = new double[]{0, 5, 8, 55};
        ySkystone2 = new double[]{0, 0, 3, 3};

        movePlatformX = new double[]{0, 24, 24};
        movePlatformY = new double[]{0, 0, 14};


        move(0, 27, -79, .8, .4, .3, "strafe", true);
        grabBlock();
        move(0, 2, 90, .5, .4, .3, "strafe", true);

        splineMove(xSkystone, ySkystone, .6, .3, .45, 0, false);
        splineMove(xPlatform, yPlatform, .45, .45, .3, 0, true);

        move(0, 3, -90, .5, .4, .3, "strafe", true);

        placeStone();

        move(0, 1, 90, .5, .4, 0.3, "strafe", true);
        grip();

        splineMove(xCrossBridge, yCrossBridge, -.8, -.35, -.5, 0, true);
        bringAlignerDown();
        releaseBlock();

        move(0, 41.5, 180, .5, .5, .3, "straight", true);

        move(0, 10, -90, .5, .4, .3, "strafe", true);

        grabBlock();
        pause(.25);

        move(0, 4, 90, .5, .4, .3, "strafe", true);

        splineMove(xSkystone2, ySkystone2, 8, .35, .65, 0, false);
        splineMove(xPlatform, yPlatform, .65, .65, .5, 0, true);
        move(0, 8, 0, .8, .5, .5, "straight", true);
        pause(.25);
        move(0, 4.5, -90, .5, .4, .3, "strafe", true);

        placeStone();

        getPlatform("left");

        grip();
        move(180, 13, -103, 0.9, .65, 0.7, "strafe", true);
        move(180, 12, 180, 0.9, .65, 0.7, "straight", true);
    }

    public void goRight() throws InterruptedException {
        xSkystone = new double[]{0, 5, 8, 34};
        ySkystone = new double[]{0, 0, 4, 4};

        xPlatform = new double[]{0, 15, 15, 42};
        yPlatform = new double[]{5, 5, 0, 0};

        xCrossBridge = new double[]{30, 15, 15, 0};
        yCrossBridge = new double[]{-3, -3, 2, 2};

        xSkystone2 = new double[]{0, 5, 8, 45};
        ySkystone2 = new double[]{0, 0, 4, 4};

        movePlatformX = new double[]{0, 22, 22};
        movePlatformY = new double[]{0, 0, 12};

        move(0, 29, -99, .6, .3, 0, "strafe", true);
        grabBlock();

        splineMove(xSkystone, ySkystone, .8, .4, .6, 0, false);
        splineMove(xPlatform, yPlatform, .6, .6, .6, 0, true);

        pause(.25);

        move(0, 1, -90, .5, .5, .5, "strafe", true);

        placeStone();

        move(0, 5.5, 90, .3, .3, 0, "strafe", true);
        grip();

        //splineMove(xCrossBridge, yCrossBridge, -.8, -.4, -.6, 0, true);

        move(0, 73, 180, .85, .4, .3, "straight", true);
        PIDTurn(180, 1);
        move(180, 16, -90, .6, .6, .6, "strafe", true);
        intake();
        moveByTime(1, .5, 180);
        stopIntake();

        move(180, 13.5, 90, .6, .6, .6, "strafe", true);
        move(180, 95, 0, .8, .6, .8, "straight", true);

        getPlatform("right");

        move(180, 4, 90, 0.85, 0.7, 0.75, "strafe", true);
        move(180, 2, 180, 0.85, 0.7, 0.77, "straight", true);
        move(180, 18, -90, 0.85, 0.7, 0.75, "strafe", true);
        move(180, 9, 180, 0.9, 0.7, 0.75, "straight", true);

        /*moveByTime(.65, -.3, 0);

        move(0, 9, -90, .5, .5, .5, "strafe", true);

        grabBlock();
        pause(.25);

        move(0, 3.5, 90, .5, .5, .5, "strafe", true);

        splineMove(xSkystone2, ySkystone2, .75, .35, .6, 0, false);
        splineMove(xPlatform, yPlatform, .6, .6, .6, 0, false);
        move(0, 2, 0, .6, .6, .45, "straight", true);
        pause(.5);

        move(0, 4, -90, .45, .45, .45, "strafe", true);
        placeStone();

        move(0, 6, 0, .6, .6, .45, "straight", true);

        getPlatform();

        grip();
        move(180, 7, -103, 0.85, 0.6, 0.7, "strafe", true);
        move(180, 16, 180, 0.85, 0.6, 0.7, "straight", true);*/
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
        pause(.25);
    }

    public void getPlatform(String blockOrientation) throws InterruptedException {
        PIDTurn(90, .75);
        if (blockOrientation.contains("right")) {
            slideAndMoveByTime(1, -.3, 90);

        }
        else {
            move(90, 3, -90, .3, .3, .3, "straight", true);
        }

        gripPlatform();
        pause(1);

        splineMove(movePlatformX, movePlatformY, .5, .3, 0, Math.PI/2, true);

        releasePlatform();
        pause(.5);
    }
}