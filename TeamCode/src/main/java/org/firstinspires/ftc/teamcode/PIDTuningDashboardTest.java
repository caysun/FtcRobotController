package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "PID Tuner", group = "7592")
public class PIDTuningDashboardTest extends AutonomousBase {
    private PIDController pid;
    private FtcDashboard dashboard;
    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap, true);

        //Initialize dashboard, controller, and dashboard + driver station telemetry (MultipleTelemetry)
        pid = new PIDController(PIDConstantsConfig.kP, PIDConstantsConfig.kI, PIDConstantsConfig.kD);
        dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        waitForStart();

        while (opModeIsActive()) {
            double currentPosition = getAngle();
            // Set motor turn power from pid controller
            double motorPower = pid.update(PIDConstantsConfig.targetPosition, currentPosition);
            robot.driveTrainTurn(motorPower);
            //Display Telemetry
            telemetry.addData("Target Position", PIDConstantsConfig.targetPosition);
            telemetry.addData("Current Position", currentPosition);
            telemetry.addData("Motor Power", motorPower);
            telemetry.update();

        }
    }
}