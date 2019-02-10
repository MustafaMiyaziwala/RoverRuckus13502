package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


@TeleOp(name="SinglesDrive", group="Linear Opmode")
public class SingleDrive extends LinearOpMode {


    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private Robot r;




    @Override
    public void runOpMode() {
        r = new Robot(hardwareMap, runtime, telemetry, this);
        r.initHardware();


        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();


        while (opModeIsActive()) {
            mecanumDrive();

            //nested ifs for intake software
           /* if (gamepad1.a && (runtime.time() - intakeSwitchTime) > 0.3) {
                intakeSwitchTime = runtime.time();
                if (!(r.intakeState == Robot.IntakeState.IN)) {
                    r.intakeState = Robot.IntakeState.IN;
                } else {
                    r.intakeState = Robot.IntakeState.STOP;
                }
            } else if (gamepad1.b && (runtime.time() - intakeSwitchTime) > 0.3) {
                intakeSwitchTime = runtime.time();
                if (!(r.intakeState == Robot.IntakeState.OUT)) {
                    r.intakeState = Robot.IntakeState.OUT;
                } else {
                    r.intakeState = Robot.IntakeState.STOP;
                }
            }

            r.setIntakeState();


            if (gamepad1.right_bumper && (runtime.time() - pivotSwitchTime) > 0.3) {
                pivotSwitchTime = runtime.time();
                r.pivotTarget = r.pivotDownPosition;
            } else if (gamepad1.left_bumper && r.pivotTarget == r.pivotDownPosition && (runtime.time() - pivotSwitchTime) > 0.3) {
                pivotSwitchTime = runtime.time();
                r.pivotTarget = r.pivotTransferPosition;
            } else if (gamepad1.left_bumper && r.pivotTarget == r.pivotTransferPosition && (runtime.time() - pivotSwitchTime) > 0.3) {
                pivotSwitchTime = runtime.time();
                r.pivotTarget = r.pivotRetractPosition;
            }

            r.setPivotState();

            //logic for dump system
            if (gamepad1.left_trigger > 0.25) {
                r.dumpTarget = r.dumpPosition;
            } else if (gamepad1.right_trigger > 0.25) {
                r.dumpTarget = r.downPosition;
            }

            r.setDumpState();

            if (gamepad1.x || gamepad2.y) {
                r.climb.setPower(1);
            } else if (gamepad1.y || gamepad2.y) {
                r.climb.setPower(-1);
            } else {
                r.climb.setPower(0);
            }

*/
            if(gamepad1.a){
                r.pulley.setPower(0.3);
            } else if(gamepad1.b){
                r.pulley.setPower(-0.3);
            }
            telemetry.addData("Intake Pivot: ", r.intakePivot.getCurrentPosition());
            telemetry.addData("Pulley: ", r.pulley.getCurrentPosition());
            telemetry.update();

            idle();

        }
    }


    public void mecanumDrive(){
        double rightX = gamepad1.right_stick_x;
        double leftX = gamepad1.left_stick_x;
        double leftY = -gamepad1.left_stick_y;

        double robotSpeed = Math.pow(leftX, 2) + Math.pow(leftY, 2);
        robotSpeed = Math.sqrt(robotSpeed);

        double changeDirectionSpeed = -rightX;

        double desiredRobotAngle = Math.atan2(-leftX, leftY);

        double frontLeft = robotSpeed * Math.sin(-desiredRobotAngle + (Math.PI / 4)) - changeDirectionSpeed;
        double frontRight = robotSpeed * Math.cos(-desiredRobotAngle + (Math.PI / 4)) + changeDirectionSpeed;
        double backLeft = robotSpeed * Math.cos(-desiredRobotAngle + (Math.PI / 4)) - changeDirectionSpeed;
        double backRight = robotSpeed * Math.sin(-desiredRobotAngle + (Math.PI / 4)) + changeDirectionSpeed;

        r.setDrivePower(frontRight, backRight, frontLeft, backLeft);

    }
}
