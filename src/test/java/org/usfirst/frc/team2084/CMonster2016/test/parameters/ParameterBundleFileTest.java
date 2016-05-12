/* 
 * Copyright (c) 2016 RobotsByTheC. All rights reserved.
 *
 * Open Source Software - may be modified and shared by FRC teams. The code must
 * be accompanied by the BSD license file in the root directory of the project.
 */
package org.usfirst.frc.team2084.CMonster2016.test.parameters;

import static org.junit.Assert.assertEquals;

import org.junit.Test;
import org.usfirst.frc.team2084.CMonster2016.parameters.Parameter;
import org.usfirst.frc.team2084.CMonster2016.parameters.Parameter.Type;
import org.usfirst.frc.team2084.CMonster2016.parameters.ParameterBundle;

/**
 * @author Ben Wolsieffer
 */
@Parameter(key = ParameterBundleTest.STRING_KEY, type = Type.STRING, stringValue = ParameterBundleTest.DEFAULT_STRING)
@Parameter(key = ParameterBundleTest.NUMBER_KEY, type = Type.NUMBER, numberValue = ParameterBundleTest.DEFAULT_NUMBER)
@Parameter(key = ParameterBundleTest.BOOLEAN_KEY, type = Type.BOOLEAN,
        booleanValue = ParameterBundleTest.DEFAULT_BOOLEAN)
public class ParameterBundleFileTest {

    public static final String STRING_KEY = "String";
    public static final String NUMBER_KEY = "Number";
    public static final String BOOLEAN_KEY = "Boolean";

    public static final String DEFAULT_STRING = "Test Value";
    public static final double DEFAULT_NUMBER = 42.24;
    public static final boolean DEFAULT_BOOLEAN = false;

    private final ParameterBundle<ParameterBundleTest> parameters =
            new ParameterBundle<>("Parameter Bundle File Test", ParameterBundleTest.class);

    @Test
    public void testStringFromFile() {
        assertEquals(parameters.getString(STRING_KEY), "a string");
    }

    @Test
    public void testNumberFromFile() {
        assertEquals(parameters.getNumber(NUMBER_KEY), 0.43, 0.01);
    }

    @Test
    public void testBooleanFromFile() {
        assertEquals(parameters.getBoolean(BOOLEAN_KEY), true);
    }

}
