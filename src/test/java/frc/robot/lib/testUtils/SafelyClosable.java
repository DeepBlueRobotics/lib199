package frc.robot.lib.testUtils;

public interface SafelyClosable extends AutoCloseable {
    // Same definition as AutoClosable but does not declare "throws Exception"
    public void close();
}
