package org.firstinspires.ftc.teamcode.ProgrammingBoards;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
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
    public static double aimStep = 1.5; //degrees per dpad press

    //pushes against the spin when the turret should stop
    public static double brakeGain = 0.2; //percent power per degree per second
    public static double brakeMaxPower = 40; //percent power cap
    public static double brakeDeadband = 3; //degrees per second, slower than this just holds

    //limelight mount, measure these on the robot
    public static boolean relocalize = true; //false ignores the limelight
    public static boolean cameraOnTurret = true; //false if it is bolted to the chassis
    public static double cameraPitch = 45; //degrees tilted up from level
    public static double cameraYaw = 0; //degrees, counterclockwise from turret forward
    public static double cameraHeight = 10; //inches, floor to lens MEASURE THIS
    public static double cameraForward = 0; //inches ahead of the pivot
    public static double cameraLeft = 0; //inches left of the pivot
    public static int pipeline = 0;

    //how much each reading moves the pose, 1 snaps right to it
    public static double relocalizeGain = 0.03;
    public static double maxCorrectionStep = 0.05; //inches, most one reading can move the pose
    public static double maxCorrectionDistance = 8; //inches, readings farther than this from odometry get ignored
    public static double maxStaleness = 100; //ms, older results get ignored
    public static double maxSpinRate = 20; //degrees per second, camera spinning faster gets ignored

    //startup sweep that finds the start location from the tags
    public static boolean findStart = false; //false skips the sweep and uses the fixed start
    public static double sweepPower = 12; //percent power
    public static double sweepLimit = 170; //degrees each way
    public static double settleTime = 0.3; //seconds to stop shaking before reading
    public static double settleTimeout = 1; //seconds without a good read before sweeping again
    public static int startSamples = 10; //readings averaged together
    public static double startMaxError = 3; //inches, worse tag fits get ignored

    //hive shape from the game manual, tags are on the bottom of each cell
    public static double hiveTilt = 25; //degrees the hive arm sits above level
    public static double hivePivotHeight = 43.95; //inches, floor to pivot
    public static double tagAlongArm = 14.27; //inches along the arm from pivot to tag center
    public static double tagBelowArm = 3; //inches from the arm line down to the tag

    //{tag id, x inches, side}, side is 1 for the far cell and -1 for the audience cell
    //tags not in here are ignored
    private static final double[][] tagPositions = {
            {33, -19.25, 1}, {32, -15.5, 1}, {31, -10, 1}, {30, -6.25, 1},
            {34, -19.25, -1}, {35, -15.5, -1}, {36, -10, -1}, {37, -6.25, -1},
            {38, 6.25, -1}, {39, 10, -1}, {40, 15.5, -1}, {41, 19.25, -1},
            {45, 6.25, 1}, {44, 10, 1}, {43, 15.5, 1}, {42, 19.25, 1},
    };

    public DcMotorEx turret;
    private final ElapsedTime timer = new ElapsedTime();
    private double lastError = 0;
    private double targetAngle = 0;
    private double aimOffset = 0; //manual aim fix, counterclockwise is positive
    private Limelight3A limelight;
    private final ElapsedTime settleTimer = new ElapsedTime();
    private final ElapsedTime lostTimer = new ElapsedTime();
    private double sweepDirection = 1;
    private boolean settling = false;
    private double lastResultTime = -1;
    private int sampleCount = 0;
    private double sumX, sumY, sumSin, sumCos;

    public Turret(HardwareMap hardwareMap) {
        turret = hardwareMap.get(DcMotorEx.class, "turret");
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //tryGet so the robot still runs with no limelight plugged in
        limelight = hardwareMap.tryGet(Limelight3A.class, "limelight");
        if (limelight != null) {
            limelight.setPollRateHz(100);
            limelight.pipelineSwitch(pipeline);
            limelight.start();
        }
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

    //stops the turret and fights any leftover spin
    public void brake() {
        double rate = getRate();
        double power = 0;
        if (Math.abs(rate) > brakeDeadband) {
            double cap = brakeMaxPower / 100;
            power = Math.max(-cap, Math.min(cap, -brakeGain / 100 * rate));
        }
        turret.setPower(power);
        lastError = 0;
        timer.reset();
    }

    //turret speed in degrees per second, counterclockwise is positive
    public double getRate() {
        double rate = turret.getVelocity() * 360.0 / ticksPerRev;
        return encoderReversed ? -rate : rate;
    }

    //turret angle in degrees from robot forward, counterclockwise is positive
    public double getAngle() {
        double degrees = turret.getCurrentPosition() * 360.0 / ticksPerRev;
        return encoderReversed ? -degrees : degrees;
    }

    //positive direction = counterclockwise
    public void adjustAim(double direction) {
        aimOffset += direction * aimStep;
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

    //call every loop, returns the pose nudged toward what the april tags say
    //heading stays from the odometry, only x and y get corrected
    public Pose2d relocalize(Pose2d pose, PoseVelocity2d robotVel) {
        if (!relocalize || limelight == null) {
            return pose;
        }

        //camera blur and lag make readings bad while spinning fast
        double turretRate = cameraOnTurret ? getRate() : 0;
        if (Math.abs(Math.toDegrees(robotVel.angVel) + turretRate) > maxSpinRate) {
            return pose;
        }

        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid() || result.getStaleness() > maxStaleness) {
            return pose;
        }

        //only use each camera reading once, odometry does the work between them
        if (result.getTimestamp() == lastResultTime) {
            return pose;
        }
        lastResultTime = result.getTimestamp();

        double heading = pose.heading.toDouble();
        double sumX = 0;
        double sumY = 0;
        int count = 0;
        for (LLResultTypes.FiducialResult tag : result.getFiducialResults()) {
            double[] tagPosition = findTag(tag.getFiducialId());
            if (tagPosition == null) {
                continue;
            }
            double[] robotPosition = closestRobotPosition(pose, tagPosition, tag.getTargetXDegrees(), tag.getTargetYDegrees());
            if (robotPosition != null
                    && Math.hypot(robotPosition[0] - pose.position.x, robotPosition[1] - pose.position.y) <= maxCorrectionDistance) {
                sumX += robotPosition[0];
                sumY += robotPosition[1];
                count++;
            }
        }
        if (count == 0) {
            return pose;
        }

        //odometry does the aiming, the limelight only gets a tiny nudge
        double stepX = relocalizeGain * (sumX / count - pose.position.x);
        double stepY = relocalizeGain * (sumY / count - pose.position.y);
        double step = Math.hypot(stepX, stepY);
        if (step > maxCorrectionStep) {
            stepX *= maxCorrectionStep / step;
            stepY *= maxCorrectionStep / step;
        }
        return new Pose2d(pose.position.x + stepX, pose.position.y + stepY, heading);
    }

    private double[] findTag(int id) {
        for (double[] tagPosition : tagPositions) {
            if ((int) tagPosition[0] == id) {
                return tagPosition;
            }
        }
        return null;
    }

    //each cell can be tipped up or down, try both and keep the answer closest to the odometry
    private double[] closestRobotPosition(Pose2d pose, double[] tagPosition, double tx, double ty) {
        double heading = pose.heading.toDouble();
        double[] best = null;
        double bestDistance = Double.MAX_VALUE;
        for (int tip = -1; tip <= 1; tip += 2) {
            //tip 1 is the raised cell, -1 is the lowered cell
            double tagY = tagPosition[2] * tagDistanceFromCenter(tip);
            double[] candidate = robotFromTag(tagPosition[1], tagY, tagHeight(tip), tx, ty, heading);
            if (candidate != null) {
                double distance = Math.hypot(candidate[0] - pose.position.x, candidate[1] - pose.position.y);
                if (distance < bestDistance) {
                    bestDistance = distance;
                    best = candidate;
                }
            }
        }
        return best;
    }

    //works backwards from where the tag is in the picture to where the robot center is
    private double[] robotFromTag(double tagX, double tagY, double tagHeight, double tx, double ty, double heading) {
        double[] ray = tagFromCamera(tagHeight, tx, ty);
        if (ray == null) {
            return null;
        }
        double forward = ray[0];
        double left = ray[1];

        //turn the camera numbers into field numbers
        double mountAngle = cameraOnTurret ? heading + Math.toRadians(getAngle()) : heading;
        double cameraAngle = mountAngle + Math.toRadians(cameraYaw);
        double cameraX = tagX - (forward * Math.cos(cameraAngle) - left * Math.sin(cameraAngle));
        double cameraY = tagY - (forward * Math.sin(cameraAngle) + left * Math.cos(cameraAngle));

        return robotFromCamera(cameraX, cameraY, mountAngle, heading);
    }

    //where the tag is from the camera, inches {forward, left} along the floor
    private double[] tagFromCamera(double tagHeight, double tx, double ty) {
        //line from the camera to the tag, tilted by the camera pitch
        double pitch = Math.toRadians(cameraPitch);
        double tanX = Math.tan(Math.toRadians(tx));
        double tanY = Math.tan(Math.toRadians(ty));
        double forward = Math.cos(pitch) - tanY * Math.sin(pitch);
        double up = Math.sin(pitch) + tanY * Math.cos(pitch);
        double left = -tanX;

        //stretch the line until it reaches the tag height
        double rise = tagHeight - cameraHeight;
        if (up * rise <= 0) {
            return null; //line never reaches the tag, bad reading
        }
        double scale = rise / up;
        if (forward * scale <= 0) {
            return null;
        }
        return new double[] {forward * scale, left * scale};
    }

    //take off the camera mount to get the robot center
    private double[] robotFromCamera(double cameraX, double cameraY, double mountAngle, double heading) {
        double robotX = cameraX - cameraForward * Math.cos(mountAngle) + cameraLeft * Math.sin(mountAngle);
        double robotY = cameraY - cameraForward * Math.sin(mountAngle) - cameraLeft * Math.cos(mountAngle);
        if (cameraOnTurret) {
            robotX -= turretOffsetX * Math.cos(heading) - turretOffsetY * Math.sin(heading);
            robotY -= turretOffsetX * Math.sin(heading) + turretOffsetY * Math.cos(heading);
        }

        //off the field means a bad reading
        if (Math.abs(robotX) > 72 || Math.abs(robotY) > 72) {
            return null;
        }
        return new double[] {robotX, robotY};
    }

    //tip 1 is the raised cell, -1 is the lowered cell
    private double tagDistanceFromCenter(int tip) {
        double tilt = Math.toRadians(hiveTilt);
        return tagAlongArm * Math.cos(tilt) + tip * tagBelowArm * Math.sin(tilt);
    }

    private double tagHeight(int tip) {
        double tilt = Math.toRadians(hiveTilt);
        return hivePivotHeight + tip * tagAlongArm * Math.sin(tilt) - tagBelowArm * Math.cos(tilt);
    }

    //true if the sweep can run
    public boolean canFindStart() {
        return findStart && limelight != null;
    }

    //call every loop at the start, sweeps until it sees tags then returns the start pose, null until done
    public Pose2d findStartPose() {
        LLResult result = limelight.getLatestResult();
        boolean fresh = result != null && result.isValid() && result.getStaleness() <= maxStaleness;
        double[] solved = fresh ? solveStart(result) : null;

        if (!settling) {
            if (solved == null) {
                sweep();
                return null;
            }
            brake();
            settling = true;
            settleTimer.reset();
            sampleCount = 0;
            sumX = sumY = sumSin = sumCos = 0;
            return null;
        }

        if (settleTimer.seconds() < settleTime) {
            brake();
            lostTimer.reset();
            return null;
        }

        //camera lost the tags, go back to sweeping
        if (solved == null) {
            if (lostTimer.seconds() > settleTimeout) {
                settling = false;
            }
            return null;
        }
        lostTimer.reset();

        if (result.getTimestamp() == lastResultTime) {
            return null;
        }
        lastResultTime = result.getTimestamp();
        sumX += solved[0];
        sumY += solved[1];
        sumSin += Math.sin(solved[2]);
        sumCos += Math.cos(solved[2]);
        sampleCount++;
        if (sampleCount < startSamples) {
            return null;
        }

        settling = false;
        return new Pose2d(sumX / sampleCount, sumY / sampleCount, Math.atan2(sumSin, sumCos));
    }

    private void sweep() {
        double angle = getAngle();
        if (angle >= sweepLimit) {
            sweepDirection = -1;
        } else if (angle <= -sweepLimit) {
            sweepDirection = 1;
        }
        turret.setPower(sweepDirection * sweepPower / 100);
    }

    //finds {x, y, heading} from the tags in one cluster, needs at least 2 tags
    private double[] solveStart(LLResult result) {
        int[] counts = new int[4];
        for (LLResultTypes.FiducialResult tag : result.getFiducialResults()) {
            double[] tagPosition = findTag(tag.getFiducialId());
            if (tagPosition != null) {
                counts[clusterOf(tagPosition)]++;
            }
        }
        int cluster = 0;
        for (int i = 1; i < counts.length; i++) {
            if (counts[i] > counts[cluster]) {
                cluster = i;
            }
        }
        if (counts[cluster] < 2) {
            return null;
        }

        //the tag spacing only fits for the right tip, so keep the better one
        double[] up = solveCluster(result, cluster, 1);
        double[] down = solveCluster(result, cluster, -1);
        double[] best = up;
        if (up == null || (down != null && down[3] < up[3])) {
            best = down;
        }
        if (best == null || best[3] > startMaxError) {
            return null;
        }
        return best;
    }

    private int clusterOf(double[] tagPosition) {
        return (tagPosition[1] < 0 ? 0 : 2) + (tagPosition[2] > 0 ? 1 : 0);
    }

    //lines up where the camera sees the tags with where they are on the field, returns {x, y, heading, error}
    private double[] solveCluster(LLResult result, int cluster, int tip) {
        java.util.List<double[]> points = new java.util.ArrayList<>(); //{field x, field y, forward, left}
        for (LLResultTypes.FiducialResult tag : result.getFiducialResults()) {
            double[] tagPosition = findTag(tag.getFiducialId());
            if (tagPosition == null || clusterOf(tagPosition) != cluster) {
                continue;
            }
            double[] ray = tagFromCamera(tagHeight(tip), tag.getTargetXDegrees(), tag.getTargetYDegrees());
            if (ray == null) {
                return null;
            }
            points.add(new double[] {tagPosition[1], tagPosition[2] * tagDistanceFromCenter(tip), ray[0], ray[1]});
        }

        double[] mean = new double[4];
        for (double[] point : points) {
            for (int i = 0; i < 4; i++) {
                mean[i] += point[i] / points.size();
            }
        }

        double cross = 0;
        double dot = 0;
        for (double[] point : points) {
            double fieldX = point[0] - mean[0];
            double fieldY = point[1] - mean[1];
            double forward = point[2] - mean[2];
            double left = point[3] - mean[3];
            cross += forward * fieldY - left * fieldX;
            dot += forward * fieldX + left * fieldY;
        }
        double cameraAngle = Math.atan2(cross, dot);
        double cos = Math.cos(cameraAngle);
        double sin = Math.sin(cameraAngle);

        double squaredError = 0;
        for (double[] point : points) {
            double forward = point[2] - mean[2];
            double left = point[3] - mean[3];
            double errorX = point[0] - mean[0] - (forward * cos - left * sin);
            double errorY = point[1] - mean[1] - (forward * sin + left * cos);
            squaredError += errorX * errorX + errorY * errorY;
        }

        double cameraX = mean[0] - (mean[2] * cos - mean[3] * sin);
        double cameraY = mean[1] - (mean[2] * sin + mean[3] * cos);
        double mountAngle = cameraAngle - Math.toRadians(cameraYaw);
        double heading = cameraOnTurret ? mountAngle - Math.toRadians(getAngle()) : mountAngle;
        double[] robot = robotFromCamera(cameraX, cameraY, mountAngle, heading);
        if (robot == null) {
            return null;
        }
        return new double[] {robot[0], robot[1], heading, Math.sqrt(squaredError / points.size())};
    }

    //call every loop to keep the turret pointed at the goal
    public void aimAtGoal(Pose2d pose, PoseVelocity2d robotVel) {
        //under the goal, just hold still
        if (inDeadzone(pose)) {
            brake();
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
        targetAngle = pickReachableAngle(Math.toDegrees(fieldAngle - heading) + aimOffset, currentAngle);

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