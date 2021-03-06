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

/*
 * This is an example LinearOpMode that shows how to use
 * the REV Robotics Color-Distance Sensor.
 *
 * It assumes the sensor is configured with the name "sensor_color_distance".
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list.
 */
@Autonomous(name = "Harvester_PlanB_Red", group = "A_Harvester_red")

public class AutoHarvesterPlanBRed extends AutoHarvesterPlanARed {

    int sideWayDistance = 5400;
    int sideWayCompensation = 900;
    double sideMovePower2 = 0;

    public AutoHarvesterPlanBRed() {

        super();

        teamColor = "red";
        fGlyphTurnAngle = 0.0f;

        vuforiaDetectingPower = -0.2;

        leftColumnDistance = 3200;
        centerColumnDistance = 2000;
        rightColumnDistance = 500;

        offBalanceStoneDistance = 2400;
        cryptoBoxDistance = 100;

        glyph2CenterDistance = 4500;

        backupDistance = 500;

        sideMovePower = -0.95;
        sideMovePower2 = sideMovePower;

        glyphDeliverPower = -0.2;

        glyphLiftPosition = 1500;

    }

    @Override
    public void loop() {
        switch (state) {
            case 0:
                robot.defaultGlyphWheelPower = 0.5;
                // jewel handling
                state = jewelKicker.loop(0, 1, teamColor);

                // hitter arm to avoid jewel holes
                jewelKicker.jewelArmActionPosition = jewelArmPos + 0.08 * rand.nextDouble() - 0.04;
                jewelKicker.jewelHitterRestPosition = jewelHitterPos + 0.02 * rand.nextDouble() - 0.01;

                robot.levelGlyph();

                computeGlyphColumnDistance();

                break;
            case 1:

                //read vumark
                double movePower = vuforiaDetectingPower;
                if ("unknown" == vuforia.vumarkImage.toLowerCase()) {
                    computeGlyphColumnDistance();
                } else {
                    movePower = vuforiaDetectingPower * 3.0;
                }

                // lift glyph bar
                VortexUtils.moveMotorByEncoder(robot.liftMotor, 10, liftMotorHolderPower);

                //set jewel hitter position
                robot.jewelHitter.setPosition(0.00);
                robot.jewelArm.setPosition(0.90);

                if (0 == moveByDistance(movePower, offBalanceStoneDistance)) {
                    moveAtPower(0.0);
                    getWheelLandmarks();
                    navigation.resetTurn(leftMotors, rightMotors);
                    state = 2;
                }

                break;
            case 2:
                // move left
                if (0 == sideMoveByDistance(sideMovePower, columnDistance)) {
                    wheelDistanceLandMark = getWheelOdometer();
                    robot.retractJewelArm();
                    state = 3;
                }

                break;
            case 3:
                // turn if necessary
                if (fGlyphTurnAngle == 0.0f || 0 == navigation.turnByEncoderOpenLoop(glyTurnPower, fGlyphTurnAngle,
                        robot.axleDistance, leftMotors, rightMotors)) {
                    getWheelLandmarks();
                    navigation.resetTurn(leftMotors, rightMotors);
                    state = 4;
                }
                break;
            case 4:

                if (0 == moveByDistance(glyphDeliverPower, cryptoBoxDistance)) {
                    timeStamp = System.currentTimeMillis();
                    state = 5;
                }

                break;
            case 5:
                // release the glyph
                time = System.currentTimeMillis();

                if (time - timeStamp < 800) {

                    robot.dumpGlyph();

                } else {

                    timeStamp = System.currentTimeMillis();
                    state = 6;
                }
                break;
            case 6:
                //back up
                if (0 == moveByDistance(-glyphDeliverPower*2, backupDistance)) {
                    timeStamp = System.currentTimeMillis();
                    robot.levelGlyph();
                    getWheelLandmarks();
                    state = 7;
                }

                break;
            case 7:
                // push
                if (0 == moveByDistance(glyphDeliverPower * 2, pushDistance + 150)) {
                    moveAtPower(0.0);
                    timeStamp = System.currentTimeMillis();
                    getWheelLandmarks();
                    navigation.resetTurn(leftMotors, rightMotors);

                    state = 21; // jump to 21 back up
                }

                break;
            case 8:
                // backup
                if (0 == moveByDistance(-glyphDeliverPower*3, 500)) {

                    moveAtPower(0.0);
                    timeStamp = System.currentTimeMillis();
                    getWheelLandmarks();
                    navigation.resetTurn(leftMotors, rightMotors);
                    // lower glyph bars
                    VortexUtils.moveMotorByEncoder(robot.liftMotor, 0, liftMotorHolderPower);
                    robot.loadGlyph();
                    state = 9;
                }

                break;
            case 9:
                // move side way
                if (0 == sideMoveByDistance(sideMovePower2, sideWayDistance - columnDistance)) {
                    wheelDistanceLandMark = getWheelOdometer();
                    getWheelLandmarks();
                    robot.glyphWheelLoad();
                    robot.extendGlyphBlocker();
                    robot.retractJewelArm();
                    state = 10;
                }

                break;
            case 10:
                robot.retractJewelArm();
                // move to center
                if (0 == moveByDistance(move2CenterPower, (int)(glyph2CenterDistance*0.75))) {
                    timeStamp = System.currentTimeMillis();
                    getWheelLandmarks();
                    state = 11;
                }
                break;
            case 11:
                // back up
                if (0 == moveByDistance(-move2CenterPower, 300)) {
                    moveAtPower(0.0);
                    navigation.resetTurn(leftMotors, rightMotors);
                    getWheelLandmarks();
                    timeStamp = System.currentTimeMillis();
                    navigation.resetTurn(leftMotors, rightMotors);
                    state = 12;
                }
                break;
            case 12:
                robot.retractJewelArm();
                navigation.turnByGyroCloseLoop(0.0,
                        (double) robot.imu.getAngularOrientation().firstAngle,
                        fGlyphTurnAngle+rand.nextInt(30)-15,
                        leftMotors, rightMotors);
                // move to center slower to collect glyph
                if (0 == moveByDistance(collectingGlyphPower, (int) (glyph2CenterDistance * 0.25)+300)) {
                    moveAtPower(0.0);
                    navigation.resetTurn(leftMotors, rightMotors);
                    getWheelLandmarks();
                    timeStamp = System.currentTimeMillis();
                    state = 13;
                }
                break;
            case 13:
                navigation.turnByGyroCloseLoop(0.0,
                        (double) robot.imu.getAngularOrientation().firstAngle,
                        fGlyphTurnAngle+rand.nextInt(30)-15,
                        leftMotors, rightMotors);
                // collect glyph
                if (System.currentTimeMillis() - timeStamp > 1000) {
                    state = 14;
                    getWheelLandmarks();
                    navigation.resetTurn(leftMotors, rightMotors);
                }
                break;
            case 14:
                // correct angle just increase it got knocked out the cource
                if (0 == navigation.turnByGyroCloseLoop(0.0,
                        (double) robot.imu.getAngularOrientation().firstAngle,
                        fGlyphTurnAngle, leftMotors, rightMotors)) {
                    state = 15;
                    robot.retractGlyphBlocker();
                    getWheelLandmarks();
                    robot.levelGlyph();
                    // lift
                    VortexUtils.moveMotorByEncoder(robot.liftMotor, glyphLiftPosition, liftMotorMovePower);
                    navigation.resetTurn(leftMotors, rightMotors);
                    timeStamp = System.currentTimeMillis();
                }
                robot.retractJewelArm();
                break;
            case 15:
                // unload
                if (System.currentTimeMillis() - timeStamp > 1000) {
                    robot.glyphWheelUnload();
                }

                // move away from center
                if (0 == moveByDistance(-move2CenterPower, glyph2CenterDistance)) {
                    timeStamp = System.currentTimeMillis();
                    getWheelLandmarks();
                    state = 16;
                }
                break;
            case 16:
                // move side way
                robot.retractJewelArm();
                if (0 == sideMoveByDistance(-sideMovePower2, sideWayDistance - columnDistance-sideWayCompensation)) {
                    wheelDistanceLandMark = getWheelOdometer();
                    getWheelLandmarks();

                    state = 17;
                }
                break;
            case 17:
                // move to glyph
                robot.retractJewelArm();
                if (0 == moveByDistance(glyphDeliverPower, backupDistance-200)) {
                    timeStamp = System.currentTimeMillis();
                    getWheelLandmarks();
                    state = 18;
                }
                break;
            case 18:
                // release glyph
                robot.dumpGlyph();
                if (System.currentTimeMillis() - timeStamp > 2000) {
                    getWheelLandmarks();
                    state = 19;
                }
                break;
            case 19:
                // back up
                if (0 == moveByDistance(-glyphDeliverPower*2, 500)) {
                    timeStamp = System.currentTimeMillis();
                    getWheelLandmarks();
                    state = 20;
                }
                break;
            case 20:
                // push
                if (0 == moveByDistance(glyphDeliverPower*2, 700)) {
                    timeStamp = System.currentTimeMillis();
                    getWheelLandmarks();
                    state = 21;
                }
                break;
            case 21:
                // back up
                if (0 == moveByDistance(-glyphDeliverPower*2, 300)) {
                    timeStamp = System.currentTimeMillis();
                    getWheelLandmarks();
                    state = 22;
                }
                break;
            default:
                robot.loadGlyph();
                VortexUtils.moveMotorByEncoder(robot.liftMotor, 0, liftMotorMovePower);
                // stop
                vuforia.relicTrackables.deactivate();
                robot.stop();
        }

        telemetry.addData("teamColor", teamColor);
        telemetry.update();
    }
}
