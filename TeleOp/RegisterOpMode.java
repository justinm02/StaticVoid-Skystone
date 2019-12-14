package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpModeManager;
import com.qualcomm.robotcore.eventloop.opmode.OpModeRegistrar;

/*
 * Created by milushev_921434 on 9/7/2018.
 */
public class RegisterOpMode {
    @OpModeRegistrar
    public static void registerMyOpMode(OpModeManager manager) {
        manager.register("GamerOp", org.firstinspires.ftc.teamcode.TeleOp.GamerOp.class);
        manager.register("DeadWheelsTest", org.firstinspires.ftc.teamcode.TeleOp.DeadWheelsTest.class);
    }
}