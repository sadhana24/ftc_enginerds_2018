package org.firstinspires.ftc.enginerds.opmodes;

import org.firstinspires.ftc.enginerds.utilities.EngiNerdsConstants;
import org.firstinspires.ftc.enginerdsmec.opmodes.AutonomousMec;

/**
 * Created by thangap on 10/26/2017.
 */
@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "RED Right - Complex", group = "RED Alliance")
//@Disabled
public class AutoRedRightAlternate extends AutonomousMec {
    @Override
    public void runOpMode() throws InterruptedException {
        super.g_AllianceColor = EngiNerdsConstants.ALLIANCE_COLOR_RED;
        super.g_DriverPosition = EngiNerdsConstants.DRIVER_POSITION_RIGHT_COMPLEX;
        super.runOpMode();
    }
}
