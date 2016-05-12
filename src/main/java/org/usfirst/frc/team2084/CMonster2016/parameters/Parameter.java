/* 
 * Copyright (c) 2016 RobotsByTheC. All rights reserved.
 *
 * Open Source Software - may be modified and shared by FRC teams. The code must
 * be accompanied by the BSD license file in the root directory of the project.
 */
package org.usfirst.frc.team2084.CMonster2016.parameters;

import java.lang.annotation.Documented;
import java.lang.annotation.ElementType;
import java.lang.annotation.Inherited;
import java.lang.annotation.Repeatable;
import java.lang.annotation.Retention;
import java.lang.annotation.RetentionPolicy;
import java.lang.annotation.Target;

/**
 * @author Ben Wolsieffer
 */
@Documented
@Repeatable(Parameters.class)
@Retention(RetentionPolicy.RUNTIME)
@Target(ElementType.TYPE)
@Inherited
public @interface Parameter {

    public enum Type {
        STRING(String.class),
        BOOLEAN(Boolean.class),
        NUMBER(Double.class);

        public final Class<?> type;

        private Type(Class<?> type) {
            this.type = type;
        }
    }

    String key();

    Type type();

    String stringValue() default "";

    double numberValue() default 0;

    boolean booleanValue() default false;
}
