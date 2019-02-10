/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="Unhook Marker Auto", group="Linear Opmode")

public class UnlatchMarkerAuto extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private Robot r;

    @Override
    public void runOpMode(){
        r = new Robot(hardwareMap, runtime, telemetry, this);
        r.initAuto();
        //r.detector.disable();
        telemetry.addData("Status", "Initialized");
        telemetry.update();


        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();


        r.markerDrop.setPosition(0);

        /*r.driveMotor(r.leftFront, 20, 0.5, 6);
        r.driveMotor(r.rightBack, 20, 0.5, 6);
        r.driveMotor(r.rightFront, 20, 0.5, 6);
        r.driveMotor(r.leftBack, 20, 1, 6);*/

        /*r.leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        r.leftBack.setPower(0.5);

        r.leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        r.leftFront.setPower(0.5);

        r.waitSeconds(3);

        r.leftFront.setPower(0);
        r.leftBack.setPower(0);*/


        //r.encoderDrive(50, 50, 0.2, 0.2, 5);
        //r.turnWithGyro(90, 0.2, 5, 5);
        /*r.descend();
        r.unhook();
        r.encoderDrive(42, 42, 0.3, 0.3,3);
        r.turnWithGyro(-120, 0.2, 5, 4);
        r.encoderDrive(-8, -8, 0.3, 0.3, 5);
        r.setDrivePower(0, 0, 0, 0);
        r.waitSeconds(0.5);
        r.markerDrop.setPosition(0.35);
        r.waitSeconds(0.5);
        r.markerDrop.setPosition(0.7);
        r.encoderDrive(70, 70, 0.3, 0.3, 10);
        //r.turnWithGyro(90, 0.2, 5, 5);
        //r.encoderDrive(-2, -2, 0.2, 0.2, 4);
        //r.turnWithGyro(135, 0.2, 4, 5);
        //r.encoderDrive(-2, -2, 0.2, 0.2, 4);
        //r.turnWithGyro(180, 0.2, 4, 8);
        //r.encoderDrive(-45, -45, 0.2, 0.2, 4);
*/

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            telemetry.addData("leftFront: ", r.leftFront.getCurrentPosition());
            telemetry.addData("rightFront: ", r.rightFront.getCurrentPosition());
            telemetry.addData("leftBack: ", r.leftBack.getCurrentPosition());
            telemetry.addData("rightFront: ", r.rightBack.getCurrentPosition());
            telemetry.update();
            // Setup a variable for each drive wheel to save power level for telemetry

            // Show the elapsed game time and wheel power.
        }


    }
}

