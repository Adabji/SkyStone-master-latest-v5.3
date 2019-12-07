package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "ButtonTest", group = "TeleOp")
public class ButtonTest extends OpMode {
    boolean previousGP2LBPos = false;
    boolean previousGP2RBPos = false;

    boolean currentGP2LBPos;
    boolean currentGP2RBPos;

    int liftstage = 0;
    int targetPos = 0;

    static final double LIFT_COUNTS_PER_MOTOR_REV = 537.6;
    static final double LIFT_DRIVE_GEAR_REDUCTION = 1.0;
    static final double LIFT_WHEEL_DIAMETER_INCHES = 1.25;
    static final double LIFT_COUNTS_PER_INCH = (LIFT_COUNTS_PER_MOTOR_REV * LIFT_DRIVE_GEAR_REDUCTION) /
            (LIFT_WHEEL_DIAMETER_INCHES * 3.1415);

    @Override
    public void init() {

    }

    @Override
    public void loop() {
        currentGP2LBPos = gamepad2.left_bumper;
        currentGP2RBPos = gamepad2.right_bumper;

        // lift - up a stage
        if (currentGP2RBPos && !previousGP2RBPos) {
            if (liftstage != 6) {
                liftstage++;
                targetPos = (int) ((liftstage * 4) * LIFT_COUNTS_PER_INCH);
            }

            previousGP2RBPos = currentGP2RBPos;
        } else {
            previousGP2RBPos = currentGP2RBPos;
        }

        //  lift - down a stage
        if (currentGP2LBPos && !previousGP2LBPos) {
            if (liftstage != 0) {
                liftstage--;
                targetPos = (int) ((liftstage * 4) * LIFT_COUNTS_PER_INCH);
            }

            previousGP2LBPos = currentGP2LBPos;
        } else {
            previousGP2LBPos = currentGP2LBPos;
        }

        telemetry.addData("PreviousLeftBumper", previousGP2LBPos);
        telemetry.addData("PreviousRightBumper", previousGP2RBPos);
        telemetry.addData("CurrentLeftBumper", currentGP2LBPos);
        telemetry.addData("CurrentRightBumper", currentGP2RBPos);
        telemetry.addData("liftstage", liftstage);
        telemetry.update();
    }
}
