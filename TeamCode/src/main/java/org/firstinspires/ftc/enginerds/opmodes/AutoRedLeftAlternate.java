package org.firstinspires.ftc.enginerds.opmodes;

import org.firstinspires.ftc.enginerds.utilities.EngiNerdsConstants;
import org.firstinspires.ftc.enginerdsmec.opmodes.AutonomousMec;

/**
 * Created by thangap on 10/26/2017.
 */
@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "RED Left - Complex", group = "RED Alliance")
//@Disabled
public class AutoRedLeftAlternate extends AutonomousMec {
    @Override
    public void runOpMode() throws InterruptedException {
        super.g_AllianceColor = EngiNerdsConstants.ALLIANCE_COLOR_RED;
        super.g_DriverPosition = EngiNerdsConstants.DRIVER_POSITION_LEFT_COMPLEX;

        super.runOpMode();
    }
}
