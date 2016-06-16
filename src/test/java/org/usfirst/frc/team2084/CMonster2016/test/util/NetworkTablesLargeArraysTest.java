/* 
 * Copyright (c) 2016 RobotsByTheC. All rights reserved.
 *
 * Open Source Software - may be modified and shared by FRC teams. The code must
 * be accompanied by the BSD license file in the root directory of the project.
 */
package org.usfirst.frc.team2084.CMonster2016.test.util;

import static org.mockito.Mockito.verify;

import org.junit.Test;
import org.junit.runner.RunWith;
import org.mockito.Mock;
import org.mockito.runners.MockitoJUnitRunner;
import org.usfirst.frc.team2084.CMonster2016.util.NetworkTablesLargeArrays;

import edu.wpi.first.wpilibj.tables.ITable;

/**
 * Tests the Base64 encoded array functionality.
 * 
 * @author Ben Wolsieffer
 */
@RunWith(MockitoJUnitRunner.class)
public class NetworkTablesLargeArraysTest {

    public static final String NUMBER_ARRAY_KEY = "Number Array";
    public static final double[] NUMBER_ARRAY = { 0.3434, 3.324, 434.45332, 2.2342, 6, 71.2233434 };
    public static final String NUMBER_ARRAY_ENCODED =
            "P9X6Q/5ckdFACpeNT987ZEB7J0DMeOn3QAHfpD/lyR1AGAAAAAAAAEBRzktCHbHC";

    @Mock
    ITable table;

    @Test
    public void testPutNumberArray() {
        NetworkTablesLargeArrays.putNumberArray(table, NUMBER_ARRAY_KEY, NUMBER_ARRAY);
        verify(table).putString(NUMBER_ARRAY_KEY, NUMBER_ARRAY_ENCODED);
    }

}
