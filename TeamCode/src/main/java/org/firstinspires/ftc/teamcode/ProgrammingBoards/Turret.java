package org.firstinspires.ftc.teamcode.ProgrammingBoards;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
public class Turret {
    //edit these live in the dashboard config tab
    public static double ticksPerRev = 1059; //full rotation (360) of turret
    public static boolean encoderReversed = false; //flip if ticks go down when turning counterclockwise

    //goal position on the field in inches, same coordinates as road runner
    public static double goalX = -16;
    public static double goalY = -13;
    //past this x the robot aims at the other goal, mirrored to (-goalX, goalY)
    public static double sideLineX = 0;

    //no aiming while the robot is under the goal, in field inches
    public static double deadzoneMinX = -16;
    public static double deadzoneMinY = -13;
    public static double deadzoneMaxX = 16;
    public static double deadzoneMaxY = 13;

    //turret pivot from robot center in inches, +x is robot forward, +y is robot left
    public static double turretOffsetX = 0;
    public static double turretOffsetY = 0;

    //turret can't go past these, in degrees from robot forward (counterclockwise is positive)
    //at 180 (facing backwards) it spins all the way around to -180
    public static double minAngle = -180;
    public static double maxAngle = 180;
    public static double wrapBuffer = 15; //degrees past the limit before it spins around, stops flip flopping at 180

    //aim gains, divided in the math so they stay normal numbers
    public static double kP = 2; //percent power per degree off
    public static double kD = 0; //damps fast changes in error
    public static double kV = 0; //helps keep up while the robot turns or drives
    public static double kS = 0; //percent power to overcome friction
    public static double tolerance = 1; //degrees, close enough to stop
    public static double maxPower = 60; //percent power cap

    public static double turretSpeed = -0.2; //manual stick speed

    public DcMotorEx turret;
    private final ElapsedTime timer = new ElapsedTime();
    private double lastError = 0;
    private double targetAngle = 0;

    public Turret(HardwareMap hardwareMap) {
        turret = hardwareMap.get(DcMotorEx.class, "turret");
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    //makes the current turret position count as 0 degrees, turret should face robot forward
    public void resetAngle() {
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lastError = 0;
    }

    //positive power = counterclockwise
    public void spinTurret(double pwr) {
        turret.setPower(pwr * turretSpeed);
        lastError = 0;
        timer.reset();
    }

    //turret angle in degrees from robot forward, counterclockwise is positive
    public double getAngle() {
        double degrees = turret.getCurrentPosition() * 360.0 / ticksPerRev;
        return encoderReversed ? -degrees : degrees;
    }

    public double getTargetAngle() {
        return targetAngle;
    }

    //goal x for whichever side the robot is on
    public static double getGoalX(Pose2d pose) {
        if (pose.position.x > sideLineX) {
            return -goalX;
        }
        return goalX;
    }

    //true when the robot is under the goal
    public static boolean inDeadzone(Pose2d pose) {
        double x = pose.position.x;
        double y = pose.position.y;
        return x >= deadzoneMinX && x <= deadzoneMaxX && y >= deadzoneMinY && y <= deadzoneMaxY;
    }

    //straight line distance from the turret to the goal in inches
    public static double getDistanceToGoal(Pose2d pose) {
        double heading = pose.heading.toDouble();
        double pivotX = pose.position.x + turretOffsetX * Math.cos(heading) - turretOffsetY * Math.sin(heading);
        double pivotY = pose.position.y + turretOffsetX * Math.sin(heading) + turretOffsetY * Math.cos(heading);
        return Math.hypot(getGoalX(pose) - pivotX, goalY - pivotY);
    }

    //call every loop to keep the turret pointed at the goal
    public void aimAtGoal(Pose2d pose, PoseVelocity2d robotVel) {
        //under the goal, just hold still
        if (inDeadzone(pose)) {
            turret.setPower(0);
            lastError = 0;
            timer.reset();
            return;
        }

        double heading = pose.heading.toDouble();
        double cos = Math.cos(heading);
        double sin = Math.sin(heading);

        //where the turret pivot is on the field
        double pivotX = pose.position.x + turretOffsetX * cos - turretOffsetY * sin;
        double pivotY = pose.position.y + turretOffsetX * sin + turretOffsetY * cos;

        double dx = getGoalX(pose) - pivotX;
        double dy = goalY - pivotY;
        double distSq = dx * dx + dy * dy;
        if (distSq < 1) {
            turret.setPower(0);
            return;
        }

        //angle to the goal on the field, minus where the robot faces, gives the turret angle
        double fieldAngle = Math.atan2(dy, dx);
        double currentAngle = getAngle();
        targetAngle = pickReachableAngle(Math.toDegrees(fieldAngle - heading), currentAngle);

        //how fast the turret needs to turn to stay on target, from the robot moving and turning
        double fieldVelX = robotVel.linearVel.x * cos - robotVel.linearVel.y * sin;
        double fieldVelY = robotVel.linearVel.x * sin + robotVel.linearVel.y * cos;
        double goalAngleRate = (dy * fieldVelX - dx * fieldVelY) / distSq;
        double turretRate = Math.toDegrees(goalAngleRate - robotVel.angVel);

        double dt = timer.seconds();
        timer.reset();
        double error = targetAngle - currentAngle;
        double derivative = (dt > 0 && dt < 0.1) ? (error - lastError) / dt : 0;
        lastError = error;

        double power = kV / 1000 * turretRate + kD / 1000 * derivative;
        if (Math.abs(error) > tolerance) {
            power += kP / 100 * error + kS / 100 * Math.signum(error);
        }

        double cap = maxPower / 100;
        turret.setPower(Math.max(-cap, Math.min(cap, power)));
    }

    //finds the version of the angle (+/- 360) inside the turret limits that's closest to where it is now
    private double pickReachableAngle(double angle, double currentAngle) {
        //it can go a little past 180 so it doesn't flip back and forth, once the goal is past the buffer it spins around
        double best = closestInRange(angle, currentAngle, minAngle - wrapBuffer, maxAngle + wrapBuffer);
        if (!Double.isNaN(best)) {
            return best;
        }
        //no version fits, so go to whichever limit is closer to the goal
        double wrapped = angle - 360 * Math.floor((angle - (minAngle + maxAngle) / 2 + 180) / 360);
        return Math.max(minAngle, Math.min(maxAngle, wrapped));
    }

    private double closestInRange(double angle, double currentAngle, double low, double high) {
        double best = Double.NaN;
        for (int k = -2; k <= 2; k++) {
            double candidate = angle + 360 * k;
            if (candidate >= low && candidate <= high
                    && (Double.isNaN(best) || Math.abs(candidate - currentAngle) < Math.abs(best - currentAngle))) {
                best = candidate;
            }
        }
        return best;
    }
}
