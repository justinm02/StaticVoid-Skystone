package org.firstinspires.ftc.teamcode.Autonomous.Blue;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.Autonomous.Auto;


@Autonomous(name = "BlueSkystonePlatform", group = "Autonomous")
public class BlueSkystonePlatform extends Auto {

    private double xSkystone[], ySkystone[];
    private double xCrossBridge[], yCrossBridge[];
    private double xPlatform[] = new double[]{0, -26, -26};
    private double yPlatform[] = new double[]{0, 0, -12};

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

            getPlatform();

            parkBot();
        }
        catch(InterruptedException e) { }
    }

    public void moveFromWall() throws InterruptedException {
        strafe(.4, 0, "straferight", 6);
        plow();
    }

    public SKYSTONE_POSITION predetermineMovement() throws InterruptedException {
        SKYSTONE_POSITION position = determineSkystonePlacement(color);

        if (position.equals(SKYSTONE_POSITION.RIGHT)) {

            xSkystone = new double[]{0, 0};
            ySkystone = new double[]{0, -4};

            xCrossBridge = new double[]{0, -8, -7};
            yCrossBridge = new double[]{0, 0, 94};

            move(0, -.3, 3, "straight");

        } else if (position.equals(SKYSTONE_POSITION.MIDDLE)) {
            xSkystone = new double[]{0, 1.5, 1.5};
            ySkystone = new double[]{0, 0, -6};

            xCrossBridge = new double[]{0, -9, -5};
            yCrossBridge = new double[]{0, 0, 88};
        }
        else {
            strafe(.3, 0, "strafeleft", 3);

            xSkystone = new double[]{0, 8, 8};
            ySkystone = new double[]{0, 0, -8};

            xCrossBridge = new double[]{0, -11, -8};
            yCrossBridge = new double[]{0, 0, 82};
        }
        return position;
    }

    public void getFirstSkystone() throws InterruptedException {

        splineMove(xSkystone, ySkystone, .35, 0); //backwards spline from cube to bridge, ending in same orientation

        move(-90, .35, 17, "straight");

        gripBlock();
        pause(1);

        plow();

        move(-90, .35, 3, "straight");
        gripBlock();
        pause(1);

        moveSlideByTicks(30, .65);
    }

    public void deliverFirstSkystone() throws InterruptedException {

        splineMove(xCrossBridge, yCrossBridge, -.7, -Math.PI / 2);
        PIDTurn(-90, 1.0);
        pause(.5);
    }

    public void getPlatform() throws InterruptedException {

        moveSlideByTicks(600, .65);
        move(-90, .6, 8, "straight");
        pause(.5);
        moveSlideByTicks(650, -.3);
        pause(.25);
        adjustClaw();

        moveByTime(.15, -90, .3);
        pause(.20);
        splineMove(xPlatform, yPlatform, -.5, -Math.PI/2);

        /*move(-90, -.5, 4, "straight");
        moveSlideByTicks(300, -.35);
        PIDTurn(90, 1.0);
        move(90, -.5, 4, "straight");
        pause(1);*/

        /*move(-90, -.3, 14, "straight");
        pause(.5);
        PIDTurn(-105, .3);
        move(-195, -.3, 8, "straight");
        while(currentAngle() > 5) {
            turn("cw", .35);
        }
        halt();*/
        moveByTime(.75, 0, .4);
        moveSlideByTicks(450, .65);
        pause(.5);
    }

    public void parkBot() throws InterruptedException {
        move(0, -.65, -4, "straight");
        strafe(.65, 0, "straferight", 20);
        moveSlideByTicks(400, -.4);
        plow();
        move(0, -.65, -30, "straight");
    }
}