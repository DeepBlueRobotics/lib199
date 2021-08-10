package frc.robot.lib.testUtils;

import java.io.ByteArrayOutputStream;
import java.io.PrintStream;

import org.junit.After;
import org.junit.AfterClass;
import org.junit.Before;
import org.junit.BeforeClass;

public class ErrStreamTest {

    public static ByteArrayOutputStream errStream = new ByteArrayOutputStream();
    private static PrintStream oldErrStream;

    @BeforeClass
    public static void logErrStream() {
        oldErrStream = System.err;
        System.setErr(new PrintStream(errStream));
        errStream.reset();
    }

    @AfterClass
    public static void restoreErrStream() {
        System.setErr(oldErrStream);
    }

    @Before
    @After
    public void clearErrStream() {
        errStream.reset();
    }

}
