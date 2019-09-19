package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.blackbox.MatchPhase;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.utils.RobotFeature;

@TeleOp(name="Mechanum Tele-op")
public class MecanumTele extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        try (Robot robot = new Robot(MatchPhase.TELEOP, this, new RobotFeature[] {})) {
            telemetry.addData("Status", "Ready to go");
            telemetry.addData("Heading offset", robot.imu.headingOffset);
            telemetry.update();

            waitForStart();
            robot.handleMatchStart();

            telemetry.addData("Status", "Running");
            telemetry.update();

            Gamepad lastGamepad1 = new Gamepad();
            Gamepad lastGamepad2 = new Gamepad();

            while (opModeIsActive()) {
                double slowMode = gamepad1.left_bumper ? .5 : 1.0;

                //Square values for finer slow control.
                double drivePower = 0.1572 * Math.pow(6.3594, Math.abs(gamepad1.left_stick_y)) * Math.signum(gamepad1.left_stick_y);
                double strafePower = -1 * 0.1572 * Math.pow(6.3594, Math.abs(gamepad1.left_stick_x)) * Math.signum(gamepad1.left_stick_x);
                double turnPower = .5 * 0.1572 * Math.pow(6.3594, Math.abs(gamepad1.right_stick_x)) * Math.signum(gamepad1.right_stick_x);

                //Controller doesn't report center as exactly 0.
                //This is NOT stall correction, see robot.setClippedMotorPower
                double deadZone = 0.13; // (Probably wrong value)

                //Complete Directional Mecanum Driving
                if (Math.abs(gamepad1.left_stick_y) > deadZone || Math.abs(gamepad1.left_stick_x) > deadZone || Math.abs(gamepad1.right_stick_x) > deadZone) {
                    //Sets up variables
                    double robotAngle = Math.atan2(drivePower, strafePower) - Math.PI / 4;

                    telemetry.addData("Input drive angle", Math.toDegrees(robotAngle));

                    double biggerStick = Math.max(Math.abs(turnPower), Math.max(Math.abs(strafePower), Math.abs(drivePower)));
                    double biggerDrive = Math.max(Math.abs(strafePower), Math.abs(drivePower));
                    double biggerValue = Math.max(Math.abs(Math.cos(robotAngle)), Math.abs(Math.sin(robotAngle)));
                    double stickMax = biggerDrive + Math.abs(turnPower);

                    // Calculates motor powers
                    double FL = biggerStick * ((Math.cos(robotAngle) / biggerValue * (biggerDrive / stickMax)) + (turnPower / stickMax));
                    double FR = biggerStick * ((Math.sin(robotAngle) / biggerValue * (biggerDrive / stickMax)) - (turnPower / stickMax));
                    double BL = biggerStick * ((Math.sin(robotAngle) / biggerValue * (biggerDrive / stickMax)) + (turnPower / stickMax));
                    double BR = biggerStick * ((Math.cos(robotAngle) / biggerValue * (biggerDrive / stickMax)) - (turnPower / stickMax));

                    //Powers Motors
                    robot.driveMotorsClipped(FL * slowMode, FR * slowMode, BL * slowMode, BR * slowMode);
                } else {
                    robot.driveMotors(0, 0, 0, 0);
                }

                robot.setDriveMotorZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

                telemetry.addData("Zero power", robot.driveMotorZeroPowerBehavior.toString());
                telemetry.addData("Heading offset", robot.imu.headingOffset);
                telemetry.addData("Heading", robot.getHeading());
                telemetry.update();

                try {
                    lastGamepad1.copy(gamepad1);
                    lastGamepad2.copy(gamepad2);
                } catch (RobotCoreException e) {
                    e.printStackTrace();
                }
            }
        }
    }
}