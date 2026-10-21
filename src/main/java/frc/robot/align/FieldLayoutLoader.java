package frc.robot.align;

import java.nio.file.Files;
import java.nio.file.Path;
import java.util.Optional;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import frc.robot.Constants.AlignConstants;

/** Loads the custom Stow and Seal AprilTag layout. Never throws; empty means "no usable layout". */
public final class FieldLayoutLoader {
    private FieldLayoutLoader() {}

    private static Optional<AprilTagFieldLayout> deployLayout;

    /** Loads the layout from {@code path}. Empty if missing, unparseable, or zero tags. */
    public static Optional<AprilTagFieldLayout> load(Path path) {
        try {
            if (path == null || !Files.isRegularFile(path)) {
                warn("field layout file not found: " + path);
                return Optional.empty();
            }
            AprilTagFieldLayout layout = new AprilTagFieldLayout(path);
            if (layout.getTags().isEmpty()) {
                warn("field layout has zero tags (placeholder?): " + path);
                return Optional.empty();
            }
            return Optional.of(layout);
        } catch (Exception | LinkageError e) {
            warn("failed to parse field layout " + path + ": " + e);
            return Optional.empty();
        }
    }

    /** Layout from the deploy directory (cached). Empty disables alignment and vision fusion. */
    public static synchronized Optional<AprilTagFieldLayout> getDeployLayout() {
        if (deployLayout == null) {
            deployLayout = load(Filesystem.getDeployDirectory().toPath()
                .resolve(AlignConstants.FIELD_LAYOUT_FILE));
        }
        return deployLayout;
    }

    public static boolean isAvailable() {
        return getDeployLayout().isPresent();
    }

    private static void warn(String msg) {
        String full = "FieldLayout: " + msg;
        try {
            DriverStation.reportWarning(full, false);
        } catch (Throwable t) {
            System.err.println("WARNING: " + full);
        }
    }
}
