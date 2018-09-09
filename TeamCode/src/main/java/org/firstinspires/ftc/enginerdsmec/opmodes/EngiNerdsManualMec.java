package org.firstinspires.ftc.enginerdsmec.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.enginerdsmec.hardware.EnginNerdsHardwareMec;


//------------------------------------------------------------------------------
// EngiNerdsManual
//------------------------------------------------------------------------------

@TeleOp(name = "EngiNerdsManualMec", group = "Mecanum")
public class EngiNerdsManualMec extends OpMode {

    public EnginNerdsHardwareMec robot = new EnginNerdsHardwareMec();

    /**
     * Perform any actions that are necessary when the OpMode is enabled.
     * The system calls this member once when the OpMode is enabled.
     */
    @Override
    public void init() {

        /**
         * Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(this, "MODE_TELEOP");

    }

    /**
     * The system calls this member repeatedly while the OpMode is running.
     */
    @Override
    public void loop () {

        //=====================================================================================
        // Gamepad1: Drive Motors
        //=====================================================================================
        // GamePad1 Controls: Manage the drive wheel motors.
        double left_stick_x_value = robot.scaleMotorPower(-gamepad1.left_stick_x);
        double left_stick_y_value = robot.scaleMotorPower(-gamepad1.left_stick_y);
        double right_stick_x_value = robot.scaleMotorPower(-gamepad1.right_stick_x);

        //robot.mecanumDrive_Cartesian(gamepad1.left_stick_x,gamepad1.left_stick_y,gamepad1.right_stick_x);
        //robot.mecanumDriveSimple(left_stick_x_value, left_stick_y_value);

        //Mecanum Drive Complex
/*        double speed = Math.hypot(-left_stick_x_value, left_stick_y_value);
        double direction = Math.atan2(left_stick_y_value, -left_stick_x_value) - Math.PI / 4;
        double rightX = right_stick_x_value;

        robot.mecanumDrive(speed, direction, rightX);*/

        /* another one to try */
        double speed = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y);
        double direction = Math.atan2(gamepad1.left_stick_y, -gamepad1.left_stick_x) - Math.PI / 4;
        double rotation = gamepad1.right_stick_x / 2; //reduce the power.
        robot.mecanumDrive(speed, direction, rotation);

        //=====================================================================================
        // Gamepad2: Lift Motor
        //=====================================================================================
        float lift_power = robot.scaleMotorPower(-gamepad2.right_stick_y);
        robot.operateLiftMotor(lift_power);

        //=====================================================================================
        // Gamepad2: Relic Arm Motor
        //=====================================================================================
        // Linear slide (Relic capture) motor is controlled with one joy stick (Pad2: Left Stick)
        float relic_arm_power = robot.scaleMotorPower(-gamepad2.left_stick_y);
        robot.setRelicArmDrivePower(relic_arm_power);

        //=====================================================================================
        // Gamepad2: Glyph Motor
        //=====================================================================================
        // Close Bottom Glyph claw (Gamepad2 -> DPAD left)
        if (gamepad2.dpad_left) {
            telemetry.addData("Glyph > ", "Rotating Glyp Motor to left");
            telemetry.update();
            robot.rotateGlyphMotor(true);
        }

        // Close Bottom Glyph claw (Gamepad2 -> DPAD right)
        if (gamepad2.dpad_right) {
            telemetry.addData("Glyph > ", "Rotating Glyp Motor to right");
            telemetry.update();
            robot.rotateGlyphMotor(false);
        }

        //=====================================================================================
        // Glyph Servos:
        //=====================================================================================
        if (gamepad2.x) {
            telemetry.addData("Glyph > ", "Opening LOWER Glyph Claws");
            telemetry.update();
            robot.openClaws("bottom");
        }
        if (gamepad2.y) {
            telemetry.addData("Glyph > ", "Closing LOWER Glyph Claws");
            telemetry.update();
            robot.closeClaws("bottom");
        }

        if (gamepad2.a) {
            telemetry.addData("Glyph > ", "Opening UPPER Glyph Claws");
            telemetry.update();
            robot.openClaws("top");
        }
        if (gamepad2.b) {
            telemetry.addData("Glyph > ", "Closing UPPER Glyph Claws");
            telemetry.update();
            robot.closeClaws("top");
        }
        if (gamepad1.x) {
            telemetry.addData("Glyph > ", "Controlled closing of ALL Glyph Claws");
            telemetry.update();
            robot.closeClawsControlled();
        }

        //Enables Breakmode
        if (gamepad1.right_bumper){
            telemetry.addData("Drive > ", "Activating brake mode");
            telemetry.update();
            robot.setBrake();
        }

        //Release Breakmode
        if (gamepad1.right_trigger > 0.5){
            telemetry.addData("Drive > ", "Releasing brake mode");
            telemetry.update();
            robot.releaseBrake();
        }


        //=====================================================================================
        // Relic Servos:
        //=====================================================================================
        // Open (raise) top Relic claw (Gamepad2 -> Right Trigger (top button))
        if (gamepad2.right_trigger > 0.5) {
            telemetry.addData("Relic > ", "Open Top");
            telemetry.update();
            robot.closeTopRelic();
        }

        // Close (bring down) top Relic claw (Gamepad2 -> Right Bumper (bottom button))
        if (gamepad2.right_bumper) {
            telemetry.addData("Relic > ", "Close Top");
            telemetry.update();
            robot.openTopRelic();
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


    } // loop

    /**
     * Perform any actions that are necessary when the OpMode is enabled.
     * The system calls this member once when the OpMode is enabled.
     */
    @Override
    public void start() {

    } // start

    /**
     * Perform any actions that are necessary when the OpMode is disabled.
     * The system calls this member once when the OpMode is disabled.
     */
    @Override
    public void stop() {

    } // stop

}
