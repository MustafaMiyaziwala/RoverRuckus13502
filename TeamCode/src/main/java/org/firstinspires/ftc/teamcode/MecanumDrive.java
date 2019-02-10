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

import org.opencv.core.Mat;


@TeleOp(name="MecanumDrive", group="Linear Opmode")
public class MecanumDrive extends LinearOpMode {


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


            telemetry.addData("Right Front: ", r.rightFront.getPower());
            telemetry.addData("Right Back: ", r.rightBack.getPower());
            telemetry.addData("Left Front: ", r.leftFront.getPower());
            telemetry.addData("Left Back: ", r.leftBack.getPower());

            telemetry.update();
            idle();

        }
    }



    public void tankDrive(){
        double leftPower = -gamepad1.left_stick_y;
        double rightPower = -gamepad1.right_stick_y;
        r.setDrivePower(rightPower, rightPower, leftPower, leftPower);
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
