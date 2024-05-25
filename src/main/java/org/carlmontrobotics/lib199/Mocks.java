package org.carlmontrobotics.lib199;

import java.lang.reflect.InvocationTargetException;
import java.lang.reflect.Method;
import java.lang.reflect.Modifier;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.stream.Collectors;

import org.mockito.MockSettings;
import org.mockito.Mockito;
import org.mockito.internal.stubbing.defaultanswers.ReturnsSmartNulls;
import org.mockito.invocation.InvocationOnMock;
import org.mockito.stubbing.Answer;

public final class Mocks {

    /**
     * Attempts to create an instance of a class in which some or all of the classes methods are replaced with a mocked implementation
     * @param <T> the class type which will be mocked
     * @param <U> the class type which will be used to provide method implementations
     * @param classToMock the class type which will be mocked
     * @param implClass the object to which to try to forward method calls
     * @param interfaces a list of interfaces which the mocked object should extend
     * @return an instance of <code>T</code> in which some or all of the classes methods are replaced with a mocked implementation from <code>U</code>
     * @see #createMock(Class, Object, boolean, Class...)
     * @see #createMock(Class, Object, Answer, Class...)
     */
    public static <T, U> T createMock(Class<T> classToMock, U implClass, Class<?>... interfaces) {
        return createMock(classToMock, implClass, true, interfaces);
    }

    /**
     * Attempts to create an instance of a class in which some or all of the classes methods are replaced with a mocked implementation
     * @param <T> the class type which will be mocked
     * @param <U> the class type which will be used to provide method implementations
     * @param classToMock the class type which will be mocked
     * @param implClass the object to which to try to forward method calls
     * @param forwardUnknownCalls whether methods which are not overridden will call their real methods
     * @param interfaces a list of interfaces which the mocked object should extend
     * @return an instance of <code>T</code> in which some or all of the classes methods are replaced with a mocked implementation from <code>U</code>
     * @see #createMock(Class, Object, Class...)
     * @see #createMock(Class, Object, Answer, Class...)
     */
    public static <T, U> T createMock(Class<T> classToMock, U implClass, boolean forwardUnknownCalls, Class<?>... interfaces) {
        return createMock(classToMock, implClass, forwardUnknownCalls ? InvocationOnMock::callRealMethod : new ReturnsSmartNulls(), interfaces);
    }

    /**
     * Attempts to create an instance of a class in which some or all of the classes methods are replaced with a mocked implementation
     * @param <T> the class type which will be mocked
     * @param <U> the class type which will be used to provide method implementations
     * @param classToMock the class type which will be mocked
     * @param implClass the object to which to try to forward method calls
     * @param defaultAnswer The answer to use when no overridden implementation is found
     * @param interfaces a list of interfaces which the mocked object should extend
     * @return an instance of <code>T</code> in which some or all of the classes methods are replaced with a mocked implementation from <code>U</code>
     * @see #createMock(Class, Object, Class...)
     * @see #createMock(Class, Object, boolean, Class...)
     */
    public static <T, U> T createMock(Class<T> classToMock, U implClass, Answer<Object> defaultAnswer, Class<?>... interfaces) {
        HashMap<Method, InvokableMethod> methods = new HashMap<>();
        for(Method m: listMethods(classToMock, interfaces)) {
            if(Modifier.isStatic(m.getModifiers())) {
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
        settings = settings.stubOnly(); // Because recording invocations would cause memory usage to grow without bound
        T mock = mock(classToMock, settings);
        return mock;
    }

    /**
     * Lists all of the methods available in the provided base class and interface classes
     * @param base The base class to search
     * @param interfaces Additional interface classes to search
     * @return An array of all the detected methods
     */
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
     * No longer does anything.
     * 
     * @param <T> The type of the mock
     * @param t The mock
     * @return The mock
     */
    @Deprecated
    public static <T> T reportMock(T t) {
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