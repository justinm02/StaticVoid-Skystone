//package org.firstinspires.ftc.teamcode.Autonomous.Blue;
//
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//
//import org.firstinspires.ftc.teamcode.Autonomous.Auto;
//
//@Autonomous(name = "BluePlatform", group = "Autonomous")
//public class BluePlatform extends Auto {
//
//    private ALLIANCE_COLOR color = ALLIANCE_COLOR.BLUE;
//
//    public void runOpMode() {
//        initialize();
//
//        waitForStart();
//
//        try {
//            move(0, 15, 180, .5, "straight");
//            move(0, 4, -90, .5, "strafe");
//            gripPlatform();
//            pause(.5);
//            move(0, 5, 0, .725, "straight");
//            PIDTurn(90, .5);
//            releasePlatform();
//            pause(.5);
//            moveByTime(2 ,-.3);
//            move(90, 15, 0, .5, "strafe");
//            move(90, 18, 90, .5, "straight");
//        }
//        catch (InterruptedException e) { }
//    }
//}