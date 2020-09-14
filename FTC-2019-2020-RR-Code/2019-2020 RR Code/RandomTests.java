package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.Autonomous.Auto;


@Autonomous(name = "JustTesting", group = "Autonomous")
public class RandomTests extends AutoMasterClass{
    private ALLIANCE_COLOR allianceColor = ALLIANCE_COLOR.RED;

    @Override
    public void runOpMode() {
        initialize();


        try {
           strafe(.3,0,"right",50);
        }
        catch (Exception e){
            telemetry.addData("Exception: ", e);
            telemetry.update();
        }

    }
}
