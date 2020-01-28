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
        ySkystone = new double[]{0, 0, 5, 5};

        xPlatform = new double[]{0, 15, 15, 36};
        yPlatform = new double[]{5, 5, 0, 0};

        xCrossBridge = new double[]{43, 15, 15, 3};
        yCrossBridge = new double[]{-3, -3, 0, 0};

        xSkystone2 = new double[]{0, 5, 8, 45};
        ySkystone2 = new double[]{0, 0, 4, 4};

        movePlatformX = new double[]{0, 20, 20};
        movePlatformY = new double[]{0, 0, 14};

        move(0, 28, -87, .8, .4, 0, "strafe", true);
        grabBlock();

        splineMove(xSkystone, ySkystone, .65, .35, .35, 0, false);
        splineMove(xPlatform, yPlatform, .35, .35, .2, 0, true);

        move(0, 3, -90, .5, .5, .4, "strafe", true);

        placeStone();

        move(0, 2, 90, .5, .5, 0, "strafe", true);

        splineMove(xCrossBridge, yCrossBridge, -.8, -.35, -.65, 0, false);
        bringAlignerDown();

        move(0, 48, 180, .65, .65, .5, "straight", true);
        moveByTime(.65, -.5, 0);

        move(0, 9, -90, .5, .5, .5, "strafe", true);

        grabBlock();
        pause(.25);

        move(0, 3.5, 90, .5, .5, .5, "strafe", true);

        splineMove(xSkystone2, ySkystone2, .75, .35, .6, 0, false);
        splineMove(xPlatform, yPlatform, .6, .6, .6, 0, false);
        move(0, 28, 0, .6, .6, .45, "straight", true);
        pause(0.25);

        move(0, 5, -90, .5, .5, .5, "strafe", true);
        placeStone();

        getPlatform();

        move(180, 8.5, -103, 0.85, 0.55, 0.7, "strafe", true);
        move(180, 12.5, 180, 0.85, 0.55, 0.7, "straight", true);
    }

    public void goLeft() throws InterruptedException {
        xSkystone = new double[]{0, 5, 8, 32};
        ySkystone = new double[]{0, 0, 4, 4};

        xPlatform = new double[]{0, 15, 15, 32};
        yPlatform = new double[]{5, 5, -2, -2};

        xCrossBridge = new double[]{43, 15, 15, 0};
        yCrossBridge = new double[]{-3, -3, 2, 2};

        xSkystone2 = new double[]{0, 5, 8, 55};
        ySkystone2 = new double[]{0, 0, 3, 3};

        movePlatformX = new double[]{0, 22, 22};
        movePlatformY = new double[]{0, 0, 14};


        move(0, 27, -73, .8, .4, .3, "strafe", true);
        grabBlock();
        move(0, 2, 90, .5, .4, .3, "strafe", true);

        splineMove(xSkystone, ySkystone, .6, .3, .35, 0, false);
        splineMove(xPlatform, yPlatform, .35, .35, 0, 0, true);

        move(0, 4, -90, .5, .4, .3, "strafe", true);

        placeStone();

        move(0, 1, 90, .5, .4, 0.3, "strafe", true);

        splineMove(xCrossBridge, yCrossBridge, -.8, -.35, -.5, 0, true);
        bringAlignerDown();

        move(0, 38, 180, .5, .5, .3, "straight", true);

        move(0, 10, -90, .5, .4, .3, "strafe", true);

        grabBlock();
        pause(.25);

        move(0, 3, 90, .5, .4, .3, "strafe", true);

        splineMove(xSkystone2, ySkystone2, .75, .35, .6, 0, false);
        splineMove(xPlatform, yPlatform, .6, .6, .45, 0, true);
        move(0, 8, 0, .8, .5, .5, "straight", true);
        pause(.25);
        move(0, 5, -90, .5, .4, .3, "strafe", true);

        placeStone();

        getPlatform();

        move(180, 12.5, -103, 0.85, 0.55, 0.7, "strafe", true);
        move(180, 15, 180, 0.85, .55, 0.7, "straight", true);
    }

    public void goRight() throws InterruptedException {
        xSkystone = new double[]{0, 5, 8, 46};
        ySkystone = new double[]{0, 0, 4, 4};

        xPlatform = new double[]{0, 15, 15, 46};
        yPlatform = new double[]{5, 5, 0, 0};

        xCrossBridge = new double[]{43, 15, 15, 0};
        yCrossBridge = new double[]{-3, -3, 2, 2};

        xSkystone2 = new double[]{0, 5, 8, 45};
        ySkystone2 = new double[]{0, 0, 4, 4};

        movePlatformX = new double[]{0, 20, 20};
        movePlatformY = new double[]{0, 0, 14};

        move(0, 29, -97, .8, .4, 0, "strafe", true);
        grabBlock();

        splineMove(xSkystone, ySkystone, .6, .35, .4, 0, false);
        splineMove(xPlatform, yPlatform, .4, .4, .25, 0, true);

        move(0, 2, -90, .3, .3, .3, "strafe", true);

        placeStone();

        move(0, 1, 90, .3, .3, 0, "strafe", true);

        splineMove(xCrossBridge, yCrossBridge, -.8, -.35, -.5, 0, true);
        bringAlignerDown();

        move(0, 48, 180, .65, .65, .3, "straight", true);
        moveByTime(.65, -.3, 0);

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

        move(180, 7, -103, 0.85, 0.55, 0.7, "strafe", true);
        move(180, 16, 180, 0.85, 0.55, 0.7, "straight", true);
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

    public void getPlatform() throws InterruptedException {
        PIDTurn(90, .75);
        move(90, 3, -90, .3, .3, .3, "straight", true);

        gripPlatform();
        pause(.5);

        splineMove(movePlatformX, movePlatformY, .5, .3, 0, Math.PI/2, true);

        releasePlatform();
        pause(.5);
    }
}