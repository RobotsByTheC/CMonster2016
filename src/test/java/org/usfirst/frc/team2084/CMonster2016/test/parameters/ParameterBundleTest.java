/* 
 * Copyright (c) 2016 RobotsByTheC. All rights reserved.
 *
 * Open Source Software - may be modified and shared by FRC teams. The code must
 * be accompanied by the BSD license file in the root directory of the project.
 */
package org.usfirst.frc.team2084.CMonster2016.test.parameters;

import static org.junit.Assert.assertEquals;
import static org.mockito.Mockito.*;

import org.junit.Test;
import org.usfirst.frc.team2084.CMonster2016.parameters.Parameter;
import org.usfirst.frc.team2084.CMonster2016.parameters.Parameter.Type;
import org.usfirst.frc.team2084.CMonster2016.parameters.ParameterBundle;

/**
 * Tests various {@link ParameterBundle} functionality.
 * 
 * @author Ben Wolsieffer
 */
@Parameter(key = ParameterBundleTest.STRING_KEY, type = Type.STRING, stringValue = ParameterBundleTest.DEFAULT_STRING)
@Parameter(key = ParameterBundleTest.NUMBER_KEY, type = Type.NUMBER, numberValue = ParameterBundleTest.DEFAULT_NUMBER)
@Parameter(key = ParameterBundleTest.BOOLEAN_KEY, type = Type.BOOLEAN,
        booleanValue = ParameterBundleTest.DEFAULT_BOOLEAN)
public class ParameterBundleTest {

    public static final String STRING_KEY = "String";
    public static final String NUMBER_KEY = "Number";
    public static final String BOOLEAN_KEY = "Boolean";

    public static final String DEFAULT_STRING = "Test Value";
    public static final double DEFAULT_NUMBER = 42.24;
    public static final boolean DEFAULT_BOOLEAN = true;

    public static final int LISTENER_TIMEOUT = 500;

    private final ParameterBundle<ParameterBundleTest> parameters =
            new ParameterBundle<>("Parameter Bundle Test", ParameterBundleTest.class);

    /**
     * 
     */
    public ParameterBundleTest() {
    }

    @Test
    public void testStringParameter() {
        assertEquals(parameters.getString(STRING_KEY), DEFAULT_STRING);
    }

    @Test
    public void testNumberParameter() {
        assertEquals(parameters.getNumber(NUMBER_KEY), DEFAULT_NUMBER, 0.01);
    }

    @Test
    public void testBooleanParameter() {
        assertEquals(parameters.getBoolean(BOOLEAN_KEY), DEFAULT_BOOLEAN);
    }

    @Test
    public void testStringKeyListener() {
        ParameterBundle.Listener listener = mock(ParameterBundle.Listener.class);

        parameters.addListener(STRING_KEY, listener);

        verify(listener, timeout(LISTENER_TIMEOUT)).parameterChanged(STRING_KEY, Type.STRING, DEFAULT_STRING);
    }

    @Test
    public void testNumberKeyListener() {
        ParameterBundle.Listener listener = mock(ParameterBundle.Listener.class);

        parameters.addListener(NUMBER_KEY, listener);

        verify(listener, timeout(LISTENER_TIMEOUT)).parameterChanged(NUMBER_KEY, Type.NUMBER, DEFAULT_NUMBER);
    }

    @Test
    public void testBooleanKeyListener() {
        ParameterBundle.Listener listener = mock(ParameterBundle.Listener.class);

        parameters.addListener(BOOLEAN_KEY, listener);

        verify(listener, timeout(LISTENER_TIMEOUT)).parameterChanged(BOOLEAN_KEY, Type.BOOLEAN, DEFAULT_BOOLEAN);
    }

    @Test
    public void testListener() {
        ParameterBundle.Listener listener = mock(ParameterBundle.Listener.class);

        parameters.addListener(listener);

        verify(listener, timeout(LISTENER_TIMEOUT)).parameterChanged(STRING_KEY, Type.STRING, DEFAULT_STRING);
        verify(listener, timeout(LISTENER_TIMEOUT)).parameterChanged(NUMBER_KEY, Type.NUMBER, DEFAULT_NUMBER);
        verify(listener, timeout(LISTENER_TIMEOUT)).parameterChanged(BOOLEAN_KEY, Type.BOOLEAN, DEFAULT_BOOLEAN);
    }

    @Test
    public void testSetString() {
        parameters.setString(STRING_KEY, "New Value");

        assertEquals(parameters.getString(STRING_KEY), "New Value");

        // Make sure it is reset back to the default
        parameters.setString(STRING_KEY, DEFAULT_STRING);
        assertEquals(parameters.getString(STRING_KEY), DEFAULT_STRING);
    }

    @Test
    public void testSetNumber() {
        parameters.setNumber(NUMBER_KEY, -1);

        assertEquals(parameters.getNumber(NUMBER_KEY), -1, 0.01);

        // Make sure it is reset back to the default
        parameters.setNumber(NUMBER_KEY, DEFAULT_NUMBER);
        assertEquals(parameters.getNumber(NUMBER_KEY), DEFAULT_NUMBER, 0.01);
    }

    @Test
    public void testSetBoolean() {
        parameters.setBoolean(BOOLEAN_KEY, !DEFAULT_BOOLEAN);

        assertEquals(parameters.getBoolean(BOOLEAN_KEY), !DEFAULT_BOOLEAN);

        // Make sure it is reset back to the default
        parameters.setBoolean(BOOLEAN_KEY, DEFAULT_BOOLEAN);
        assertEquals(parameters.getBoolean(BOOLEAN_KEY), DEFAULT_BOOLEAN);
    }

}
