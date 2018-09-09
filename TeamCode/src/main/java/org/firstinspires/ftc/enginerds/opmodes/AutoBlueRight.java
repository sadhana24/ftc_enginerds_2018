package org.firstinspires.ftc.enginerds.opmodes;

import org.firstinspires.ftc.enginerds.utilities.EngiNerdsConstants;
import org.firstinspires.ftc.enginerdsmec.opmodes.AutonomousMec;

/**
 * Created by thangap on 10/26/2017.
 */
@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "BLUE Right - Simple", group = "BLUE Alliance")
//@Disabled
public class AutoBlueRight extends AutonomousMec {
    @Override
    public void runOpMode() throws InterruptedException {
        super.g_AllianceColor = EngiNerdsConstants.ALLIANCE_COLOR_BLUE;
        super.g_DriverPosition = EngiNerdsConstants.DRIVER_POSITION_RIGHT_SIMPLE;
        super.runOpMode();
    }
}
