package org.firstinspires.ftc.enginerds.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.enginerds.hardware.EnginNerdsHardware;


//------------------------------------------------------------------------------
// EngiNerdsManual
//------------------------------------------------------------------------------

@TeleOp(name = "EngiNerdsManual", group = "Mecanum")
public class EngiNerdsManual extends OpMode {

    public EnginNerdsHardware robot = new EnginNerdsHardware();

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

    } // init

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

        /*
        //Mecanum Drive Complex
        double speed = Math.hypot(-left_stick_x_value, left_stick_y_value);
        double direction = Math.atan2(left_stick_y_value, -left_stick_x_value) - Math.PI / 4;
        double rightX = right_stick_x_value;
        robot.mecanumDrive(speed, direction, rightX);
        */

        /* another one to try */
        double speed = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y);
        double direction = Math.atan2(gamepad1.left_stick_y, -gamepad1.left_stick_x) - Math.PI / 4;
        double rotation = gamepad1.right_stick_x / 2; //reduce the power.
        robot.mecanumDrive(speed, direction, rotation);

        //=====================================================================================
        // Glyph Servos:
        //=====================================================================================
        if (gamepad2.x) {
            telemetry.addData("Gamepad 2 ", "X pressed");
            telemetry.update();
        }
        if (gamepad2.y) {
            telemetry.addData("Gamepad 2 ", "Y pressed");
            telemetry.update();
        }

        if (gamepad2.a) {
            telemetry.addData("Gamepad 2", "A pressed");
            telemetry.update();
        }
        if (gamepad2.b) {
            telemetry.addData("Gamepad 2", "B pressed");
            telemetry.update();
        }
        if (gamepad1.x) {
            telemetry.addData("Gamepad 1", "X pressed");
            telemetry.update();
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

        if (gamepad2.right_trigger > 0.5) {
            telemetry.addData("Gamepad 2", "Right Trigger Pressed");
            telemetry.update();
        }

        if (gamepad2.right_bumper) {
            telemetry.addData("Gamepad 2", "Right Bumper Pressed");
            telemetry.update();
        }

        if (gamepad2.left_trigger > 0.5) {
            telemetry.addData("Gamepad 2", "Left Trigger Pressed");
            telemetry.update();
        }

        if (gamepad2.left_bumper) {
            telemetry.addData("Gamepad 2", "Left Bumper Pressed");
            telemetry.update();
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
