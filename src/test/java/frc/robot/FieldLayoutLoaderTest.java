package frc.robot;

import static org.junit.jupiter.api.Assertions.*;

import java.nio.file.Files;
import java.nio.file.Path;
import java.util.Optional;

import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.io.TempDir;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import frc.robot.align.FieldLayoutLoader;

class FieldLayoutLoaderTest {
    @TempDir Path tmp;

    @Test
    void missingFileIsEmpty() {
        assertTrue(FieldLayoutLoader.load(tmp.resolve("nope.json")).isEmpty());
    }

    @Test
    void malformedFileIsEmpty() throws Exception {
        Path p = tmp.resolve("bad.json");
        Files.writeString(p, "not json");
        assertTrue(FieldLayoutLoader.load(p).isEmpty());
    }

    @Test
    void emptyTagListIsEmpty() throws Exception {
        Path p = tmp.resolve("empty.json");
        Files.writeString(p, "{\"tags\":[],\"field\":{\"length\":16.0,\"width\":8.0}}");
        assertTrue(FieldLayoutLoader.load(p).isEmpty());
    }

    @Test
    void shippedPlaceholderIsEmpty() {
        Path p = Path.of("src", "main", "deploy", "stow_and_seal_tags.json");
        assertTrue(Files.exists(p));
        assertTrue(FieldLayoutLoader.load(p).isEmpty());
    }

    @Test
    void oneTagFileLoads() throws Exception {
        Path p = tmp.resolve("one.json");
        Files.writeString(p, "{\"tags\":[{\"ID\":7,\"pose\":{\"translation\":{\"x\":1.0,\"y\":2.0,\"z\":0.5},"
            + "\"rotation\":{\"quaternion\":{\"W\":1.0,\"X\":0.0,\"Y\":0.0,\"Z\":0.0}}}}],"
            + "\"field\":{\"length\":16.0,\"width\":8.0}}");
        Optional<AprilTagFieldLayout> l = FieldLayoutLoader.load(p);
        assertTrue(l.isPresent());
        assertEquals(1, l.get().getTags().size());
        assertTrue(l.get().getTagPose(7).isPresent());
    }
}
