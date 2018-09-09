package org.firstinspires.ftc.enginerds.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.enginerds.hardware.EnginNerdsHardware;

//------------------------------------------------------------------------------
// EngiNerdsManual
//------------------------------------------------------------------------------

@TeleOp(name = "EngiNerdsManual", group = "Red Alliance")
@Disabled
public class EngiNerdsManual extends OpMode {

    public EnginNerdsHardware robot = new EnginNerdsHardware();
    /**
     * Lift Positions: 0- Initial, 1 - 2" above ground, 2 - 8" above ground, 3- 14" above ground.
     */
    private int liftPosition = 0; //Initial Position
    private boolean aDouble = false;
    private boolean bDouble = false;

    /**
     * Perform any actions that are necessary when the OpMode is enabled.
     * The system calls this member once when the OpMode is enabled.
     */
    @Override
    public void init() {

        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        // Disable motor encoders on both Drive and Lift motors
        robot.setDriveWithoutEncoder();
        robot.setLiftWithoutEncoder();
    }

    /**
     * The system calls this member repeatedly while the OpMode is running.
     */
    @Override
    public void loop () {

        /**
         * Robot Drive Operation
         * If 'x' is pressed on gamepad 2, open both claws (run servos open position)
         * If 'y' is pressed on gamepad 2, close both claws (run servos close position)
         */

        // GamePad1 Controls: Manage the drive wheel motors.
        float left_drive_power = robot.scaleMotorPower(-gamepad1.left_stick_y);
        float right_drive_power = robot.scaleMotorPower(-gamepad1.right_stick_y);
        robot.setDrivePower(left_drive_power, right_drive_power);

        // Lift motor is controlled with one joy stick (Pad2: Right Stick)
        float lift_power = robot.scaleMotorPower(-gamepad2.right_stick_y);
        robot.setLiftMotorPower(lift_power);

        // Linear slide (Relic capture) motor is controlled with one joy stick (Pad2: Left Stick)
        float arm_power = robot.scaleMotorPower(-gamepad2.left_stick_y);
        robot.setArmMotorPower(arm_power);

        // Open Glyph claws (Gamepad2 -> X button)
        if (gamepad2.x) {
            telemetry.addData("Claw > ", "Opening");
            telemetry.update();
            robot.openClaw();
        }

        // Close Glyph claws(Gamepad2 -> Y button)
        if (gamepad2.y) {
            telemetry.addData("Claw > ", "Closing");
            telemetry.update();
            robot.closeClaw();
        }

        // Partial open Glyph claws (Gamepad2 -> A button)
        if (gamepad2.a) {
            telemetry.addData("Claw > ", "Partially Closing");
            telemetry.update();
            robot.partialOpenClaw();
        }

        // Open (raise) top Relic claw (Gamepad2 -> Right Trigger (top button))
        if (gamepad2.right_trigger > 0.5) {
            telemetry.addData("Relic > ", "Open Top");
            telemetry.update();
            robot.openTopRelic();
        }

        // Close (bring down) top Relic claw (Gamepad2 -> Right Bumper (bottom button))
        if (gamepad2.right_bumper) {
            telemetry.addData("Relic > ", "Close Top");
            telemetry.update();
            robot.closeTopRelic();
        }

        // Close bottom Relic claw (Gamepad2 -> Left Trigger (top button))
        if (gamepad2.left_trigger > 0.5) {
            telemetry.addData("Relic > ", "Close Bottom");
            telemetry.update();
            robot.closeBottomRelic();
        }

        // Release bottom Relic claw (Gamepad2 -> Left Bumper (bottom button))
        if (gamepad2.left_bumper) {
            telemetry.addData("Relic > ", "Open Bottom");
            telemetry.update();
            robot.releaseBottomRelic();
        }

        // Test Closing Jewel (Gamepad1 -> X button)
        if (gamepad1.x) {
            telemetry.addData("Jewel > ", "Closing");
            telemetry.update();
            robot.jewelArmClose();
        }

        // Opening Jewel (Gamepad1 -> Y button)
        if (gamepad1.y) {
            telemetry.addData("Jewel > ", "Opening");
            telemetry.update();
            robot.jewelArmOpen();
        }

        //Enables Breakmode
        if (gamepad1.right_bumper){
            robot.setBrake();
        }

        //Release Breakmode
        if (gamepad1.right_trigger > 0.5){
            robot.releaseBrake();
        }

    } // loop

    /**
     * Perform any actions that are necessary when the OpMode is enabled.
     * <p/>
     * The system calls this member once when the OpMode is enabled.
     */
    @Override
    public void start() {

    } // start

    /**
     * Perform any actions that are necessary when the OpMode is disabled.
     * <p/>
     * The system calls this member once when the OpMode is disabled.
     */
    @Override
    public void stop() {

    } // stop

} // EngiNerdsManual
