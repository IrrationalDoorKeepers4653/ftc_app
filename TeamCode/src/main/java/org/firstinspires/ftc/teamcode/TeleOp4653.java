package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by silkenhons on 10/17/2017.
 */

public class TeleOp4653 {
    /* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
*/

    @TeleOp(name="Basic: TeleOp4653", group="Iterative Opmode")

    public class BasicOpMode_Iterative extends OpMode
    {
        // Declare OpMode members.
        private ElapsedTime runtime = new ElapsedTime();
        private DcMotor leftDrive = null;
        private DcMotor rightDrive = null;
        private DcMotor chainDrive = null;

        Servo servoArmLeft = null;
        Servo servoArmRight = null;

        public double ARM_MAX = 0.5;
        public double ARM_MIN = 0.0;

        Servo servoStinger = null;

        public double STINGER_MAX = 1.0;
        public double STINGER_MIN = 0.0;

        /*
         * Code to run ONCE when the driver hits INIT
         */
        @Override
        public void init() {
            telemetry.addData("Status", "Initialized");

            // Initialize the hardware variables. Note that the strings used here as parameters
            // to 'get' must correspond to the names assigned during the robot configuration
            // step (using the FTC Robot Controller app on the phone).
            leftDrive  = hardwareMap.get(DcMotor.class, "left_drive");
            rightDrive = hardwareMap.get(DcMotor.class, "right_drive");
            chainDrive = hardwareMap.get(DcMotor.class, "chain_drive");

            servoArmLeft = hardwareMap.get(Servo.class, "left_arm");
            servoArmRight = hardwareMap.get(Servo.class, "right_arm");

            servoStinger = hardwareMap.get(Servo.class, "stinger");

            // Most robots need the motor on one side to be reversed to drive forward
            // Reverse the motor that runs backwards when connected directly to the battery
            leftDrive.setDirection(DcMotor.Direction.FORWARD);
            rightDrive.setDirection(DcMotor.Direction.REVERSE);
            chainDrive.setDirection(DcMotor.Direction.REVERSE);

            // Tell the driver that initialization is complete.
            telemetry.addData("Status", "Initialized");
        }

        /*
         * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
         */
        @Override
        public void init_loop() {
        }

        /*
         * Code to run ONCE when the driver hits PLAY
         */
        @Override
        public void start() {
            runtime.reset();
        }

        /*
         * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
         */
        @Override
        public void loop() {
            // Setup a variable for each drive wheel to save power level for telemetry
            double leftPower;
            double rightPower;

            leftPower  = -gamepad1.left_stick_y ;
            rightPower = -gamepad1.right_stick_y ;

            // Send calculated power to wheels
            leftDrive.setPower(leftPower);
            rightDrive.setPower(rightPower);

            //turn the chain motor to raise and lower the arms
            double chainDown = gamepad1.right_trigger;
            chainDrive.setPower(chainDown);
            double chainUp = -gamepad1.left_trigger;
            chainDrive.setPower(chainUp);

            //open and close the arms on servos
            if (gamepad1.right_bumper) {                 //code to open arms
                servoArmLeft.setPosition(ARM_MAX);
                servoArmRight.setPosition(ARM_MAX);
            }
            else if (gamepad1.left_bumper){              //code to close arms
                servoArmLeft.setPosition(ARM_MIN);
                servoArmRight.setPosition(ARM_MIN);
            }
            // insert code for arms to close

            //raise and lower the stinger, can do this on a dual button, or while a single button is pressed
            if (gamepad1.y){                            //raise the stinger
                servoStinger.setPosition(STINGER_MAX);
            }
            else if (gamepad1.a){                       //lower the stinger
                servoStinger.setPosition(STINGER_MIN);
            }

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
        }

        /*
         * Code to run ONCE after the driver hits STOP
         */
        @Override
        public void stop() {
        }

    }

}
