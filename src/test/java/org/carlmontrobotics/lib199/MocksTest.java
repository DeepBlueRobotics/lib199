package org.carlmontrobotics.lib199;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import org.junit.Test;

public class MocksTest {

    @Test
    public void testBase() {
        TestBase base = new TestBase();
        assertEquals(1, base.test1());
        assertEquals(2, base.test2());
        assertEquals(3, base.test3());
    }

    @Test
    public void testSimpleMock() {
        TestBase mock = Mocks.createMock(TestBase.class, new TestImpl());
        assertFalse(mock instanceof TestInterface);
        assertEquals(1, mock.test1());
        assertEquals(3, mock.test2());
        assertEquals(4, mock.test3());
    }

    @Test
    public void testInterfaceMock() {
        TestBase mock = Mocks.createMock(TestBase.class, new TestImpl(), TestInterface.class);
        assertTrue(mock instanceof TestInterface);
        assertEquals(5, ((TestInterface)mock).test4());
    }

    @Test
    public void testNullDefaultAnswer() {
        TestBase mock = Mocks.createMock(TestBase.class, new TestImpl(), false);
        assertEquals(0, mock.test1());
        assertEquals(3, mock.test2());
        assertEquals(4, mock.test3());
    }

    @Test
    public void testCustomDefaultAnswer() {
        TestBase mock = Mocks.createMock(TestBase.class, new TestImpl(), invocation -> 5);
        assertEquals(5, mock.test1());
        assertEquals(3, mock.test2());
        assertEquals(4, mock.test3());
    }
    
    public class TestBase {
        public int test1() {
            return 1;
        }

        public int test2() {
            return 2;
        }

        public final int test3() {
            return 3;
        }
    }
    
    public interface TestInterface {
        public int test4();
    }
    
    public class TestImpl {
        public int test2() {
            return 3;
        }

        public int test3() {
            return 4;
        }

        public int test4() {
            return 5;
        }
    }

}
