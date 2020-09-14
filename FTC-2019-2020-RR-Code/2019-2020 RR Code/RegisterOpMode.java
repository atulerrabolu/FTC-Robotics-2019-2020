package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpModeManager;
import com.qualcomm.robotcore.eventloop.opmode.OpModeRegistrar;

/*

 */
public class RegisterOpMode {
    @OpModeRegistrar
    public static void registerMyOpMode(OpModeManager manager) {
        manager.register("TeleOp", org.firstinspires.ftc.teamcode.TeleOp.class);
    }
}