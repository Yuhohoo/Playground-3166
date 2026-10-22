package frc.robot.sim;

import java.nio.file.Files;
import java.nio.file.Path;
import java.util.regex.Pattern;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

/** Kinematic model, not a Phoenix/module/traction simulation. Reads limits without initializing CAN. */
final class SimRobot {
    static double read(String file, String regex) throws Exception {
        var match = Pattern.compile(regex).matcher(Files.readString(Path.of(file)));
        if (!match.find()) throw new IllegalStateException("Drive configuration changed: " + regex);
        return Double.parseDouble(match.group(1));
    }
    final double maxSpeed = read("src/main/java/frc/robot/generated/TunerConstants.java",
        "kSpeedAt12Volts = MetersPerSecond.of\\(([0-9.]+)\\)")
        * read("src/main/java/frc/robot/RobotContainer.java", "MaxSpeed = ([0-9.]+) \\*");
    final double maxOmega = read("src/main/java/frc/robot/RobotContainer.java",
        "MaxAngularRate = RotationsPerSecond.of\\(([0-9.]+)\\)") * 2 * Math.PI;
    final double manualDeadband = maxSpeed * read("src/main/java/frc/robot/RobotContainer.java",
        "withDeadband\\(MaxSpeed \\* ([0-9.]+)\\)");
    Pose2d pose;
    ChassisSpeeds speed = new ChassisSpeeds();
    SimRobot(Pose2d initial) throws Exception {
        pose = initial;
        // There are no SlewRateLimiter instances in the drive path. Deadband belongs only to
        // RobotContainer's manual request; AlignToPegCommand creates its own request with zero deadband.
        if (Files.readString(Path.of("src/main/java/frc/robot/RobotContainer.java")).contains("SlewRateLimiter"))
            throw new IllegalStateException("Update model for new drive slew limiter");
    }
    static double approach(double a, double b, double delta) { return a + Math.max(-delta, Math.min(delta, b-a)); }
    void step(ChassisSpeeds requested, double dt) {
        double norm = Math.hypot(requested.vxMetersPerSecond, requested.vyMetersPerSecond);
        double scale = Math.min(1, maxSpeed / Math.max(norm, 1e-9));
        // Hypothetical physical acceleration, not an existing drivetrain limiter.
        speed = new ChassisSpeeds(approach(speed.vxMetersPerSecond, requested.vxMetersPerSecond*scale, 3*dt),
            approach(speed.vyMetersPerSecond, requested.vyMetersPerSecond*scale, 3*dt),
            approach(speed.omegaRadiansPerSecond, Math.max(-maxOmega, Math.min(maxOmega, requested.omegaRadiansPerSecond)), 10*dt));
        pose = new Pose2d(pose.getX()+speed.vxMetersPerSecond*dt, pose.getY()+speed.vyMetersPerSecond*dt,
            new Rotation2d(pose.getRotation().getRadians()+speed.omegaRadiansPerSecond*dt));
    }
}
