package frc.robot.lib;

import java.lang.ref.WeakReference;
import java.lang.reflect.InvocationTargetException;
import java.lang.reflect.Method;
import java.lang.reflect.Modifier;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.HashMap;
import java.util.List;
import java.util.function.Consumer;
import java.util.function.Predicate;
import java.util.stream.Collectors;

import org.mockito.MockSettings;
import org.mockito.Mockito;
import org.mockito.internal.stubbing.defaultanswers.ReturnsSmartNulls;
import org.mockito.invocation.InvocationOnMock;
import org.mockito.stubbing.Answer;

public final class Mocks {

    private static final List<WeakReference<Object>> MOCKS = Collections.synchronizedList(new ArrayList<>());
    private static final Predicate<WeakReference<?>> IS_REFERENCE_CLEARED = reference -> reference.get() == null;
    private static final Consumer<WeakReference<Object>> CLEAR_INVOCATIONS_ON_REFERENCED_MOCK = reference -> Mockito.clearInvocations(reference.get());
    private static final Predicate<WeakReference<Object>> CLEAR_INVOCATIONS_ON_REFERENCED_MOCK_IF_REFERNCE_NOT_CLEARED = reference -> {
        if(IS_REFERENCE_CLEARED.test(reference)) return true;
        CLEAR_INVOCATIONS_ON_REFERENCED_MOCK.accept(reference);
        return false;
    };

    static {
        // Use a single predicate so that clearing references and invocations is an atomic operation
        // Otherwise, we could (rarely) run into:
        // 1) Mock is added
        // 2) Garbage collected references are removed
        // 3) Mock is garbage collected
        // 4) Mock invocations are cleared -> throws NullPointerException
        Lib199Subsystem.registerPeriodic(() -> MOCKS.removeIf(CLEAR_INVOCATIONS_ON_REFERENCED_MOCK_IF_REFERNCE_NOT_CLEARED));
    }
    
    /**
     * Attempts to create an instance of a class in which some or all of the classes methods are replaced with a mocked implementation
     * @param T the class type which will be mocked
     * @param U the class type which will be used to provide method implementations
     * @param classToMock the class type which will be mocked
     * @param implClass the object to which to try to forward method calls
     * @param interfaces a list of interfaces which the mocked object should extend
     * @return an instance of <code>T</code> in which some or all of the classes methods are replaced with a mocked implementation from <code>U</code>
     * @see #createMock(java.lang.Class, java.lang.Object, java.lang.Class...) 
     */
    public static <T, U> T createMock(Class<T> classToMock, U implClass, Class<?>... interfaces) {
        return createMock(classToMock, implClass, true, interfaces);
    }
    
    /**
     * Attempts to create an instance of a class in which some or all of the classes methods are replaced with a mocked implementation
     * @param T the class type which will be mocked
     * @param U the class type which will be used to provide method implementations
     * @param classToMock the class type which will be mocked
     * @param implClass the object to which to try to forward method calls
     * @param forwardUnknownCalls whether methods which are not overriden will call their real methods
     * @param interfaces a list of interfaces which the mocked object should extend
     * @return an instance of <code>T</code> in which some or all of the classes methods are replaced with a mocked implementation from <code>U</code>
     * @see #createMock(java.lang.Class, java.lang.Object) 
     */
    public static <T, U> T createMock(Class<T> classToMock, U implClass, boolean forwardUnknownCalls, Class<?>... interfaces) {
        return createMock(classToMock, implClass, forwardUnknownCalls ? InvocationOnMock::callRealMethod : new ReturnsSmartNulls(), interfaces);
    }
    
    /**
     * Attempts to create an instance of a class in which some or all of the classes methods are replaced with a mocked implementation
     * @param T the class type which will be mocked
     * @param U the class type which will be used to provide method implementations
     * @param classToMock the class type which will be mocked
     * @param implClass the object to which to try to forward method calls
     * @param defaultAnswer The answer to use when no overriden implementation is found
     * @param interfaces a list of interfaces which the mocked object should extend
     * @return an instance of <code>T</code> in which some or all of the classes methods are replaced with a mocked implementation from <code>U</code>
     * @see #createMock(java.lang.Class, java.lang.Object) 
     */
    public static <T, U> T createMock(Class<T> classToMock, U implClass, Answer<Object> defaultAnswer, Class<?>... interfaces) {
        HashMap<Method, InvokableMethod> methods = new HashMap<>();
        for(Method m: listMethods(classToMock, interfaces)) {
            if(Modifier.isStatic(m.getModifiers()) || Modifier.isFinal(m.getModifiers())) {
                continue;
            }
            try {
                Method mImpl = implClass.getClass().getMethod(m.getName(), m.getParameterTypes());
                if(!m.getReturnType().isAssignableFrom(mImpl.getReturnType())) {
                    System.err.println("Method Return Types Not the Same for Method: " + m.getName());
                }
                methods.put(m, mImpl::invoke);
            } catch(NoSuchMethodException e) {}
        }
        MockSettings settings;
        if(interfaces.length == 0) {
            settings = Mockito.withSettings();
        } else {
            settings = Mockito.withSettings().extraInterfaces(interfaces);
        }
        settings = settings.defaultAnswer(new MockAnswer<>(methods, implClass, defaultAnswer));
        T mock = mock(classToMock, settings);
        return mock;
    }
    
    public static Method[] listMethods(Class<?> base, Class<?>... interfaces) {
        ArrayList<Method> out = new ArrayList<>();
        out.addAll(Arrays.asList(base.getMethods()));
        out.addAll(Arrays.stream(interfaces).map(Class::getMethods).flatMap(Arrays::stream).collect(Collectors.toList()));
        return out.toArray(Method[]::new);
    }

    /**
     * A wrapper for the underlying Mockito method which automatically calls {@link Mockito#clearInvocations(Object...)} to prevent memory leaks
     * 
     * @see Mockito#mock(Class)
     */
    public static <T> T mock(Class<T> classToMock) {
        return reportMock(Mockito.mock(classToMock));
    }

    /**
     * A wrapper for the underlying Mockito method which automatically calls {@link Mockito#clearInvocations(Object...)} to prevent memory leaks
     * 
     * @see Mockito#mock(Class, String)
     */
    public static <T> T mock(Class<T> classToMock, String name) {
        return reportMock(Mockito.mock(classToMock, name));
    }

    /**
     * A wrapper for the underlying Mockito method which automatically calls {@link Mockito#clearInvocations(Object...)} to prevent memory leaks
     * 
     * @see Mockito#mock(Class, Answer)
     */
    public static <T> T mock(Class<T> classToMock, Answer<?> defaultAnswer) {
        return reportMock(Mockito.mock(classToMock, defaultAnswer));
    }

    /**
     * A wrapper for the underlying Mockito method which automatically calls {@link Mockito#clearInvocations(Object...)} to prevent memory leaks
     * 
     * @see Mockito#mock(Class, MockSettings)
     */
    public static <T> T mock(Class<T> classToMock, MockSettings mockSettings) {
        return reportMock(Mockito.mock(classToMock, mockSettings));
    }

    /**
     * Registers a Mockito mock and periodically calls {@link Mockito#clearInvocations(Object...)} on it to prevent memory leaks
     * 
     * @param <T> The type of the mock
     * @param t The mock
     * @return The mock
     */
    public static <T> T reportMock(T t) {
        // Wrap in a WeakReference to prevent memory leaks on objects with no more references
        if(Mockito.mockingDetails(t).isMock()) MOCKS.add(new WeakReference<Object>(t));
        return t;
    }

    private Mocks() {}

    private static final class MockAnswer<U> implements Answer<Object> {
        private final HashMap<Method, InvokableMethod> methods;
        private final U impl;
        private final Answer<?> defaultAnswer;
        MockAnswer(HashMap<Method, InvokableMethod> methods, U impl, Answer<?> defaultAnswer) {
            this.methods = methods;
            this.impl = impl;
            this.defaultAnswer = defaultAnswer;
        }
        @Override
        public Object answer(InvocationOnMock invocation) throws Throwable {
            if(methods.containsKey(invocation.getMethod())) {
                try {
                    return methods.get(invocation.getMethod()).invoke(impl, invocation.getArguments());
                } catch(InvocationTargetException e) {
                    throw e.getTargetException();
                }
            }
            return defaultAnswer.answer(invocation);
        }
    }

    private static interface InvokableMethod {
        public Object invoke(Object object, Object[] args) throws IllegalAccessException, IllegalArgumentException, InvocationTargetException;
    } 

}