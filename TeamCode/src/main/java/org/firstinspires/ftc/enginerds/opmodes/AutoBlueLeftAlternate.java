package org.firstinspires.ftc.enginerds.opmodes;

import org.firstinspires.ftc.enginerds.utilities.EngiNerdsConstants;
import org.firstinspires.ftc.enginerdsmec.opmodes.AutonomousMec;

/**
 * Created by thangap on 12/12/2017.
 */
@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "BLUE Left - Complex", group = "BLUE Alliance")
//@Disabled
public class AutoBlueLeftAlternate extends AutonomousMec {
    @Override
    public void runOpMode() throws InterruptedException {
        super.g_AllianceColor = EngiNerdsConstants.ALLIANCE_COLOR_BLUE;
        super.g_DriverPosition = EngiNerdsConstants.DRIVER_POSITION_LEFT_COMPLEX;
        super.runOpMode();
    }
}