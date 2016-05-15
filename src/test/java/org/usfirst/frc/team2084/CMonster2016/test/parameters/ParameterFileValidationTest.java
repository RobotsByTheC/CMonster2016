/* 
 * Copyright (c) 2016 RobotsByTheC. All rights reserved.
 *
 * Open Source Software - may be modified and shared by FRC teams. The code must
 * be accompanied by the BSD license file in the root directory of the project.
 */
package org.usfirst.frc.team2084.CMonster2016.test.parameters;

import static org.hamcrest.Matchers.*;
import static org.junit.Assert.assertThat;

import java.io.IOException;
import java.io.InputStream;
import java.net.URISyntaxException;
import java.nio.file.FileSystem;
import java.nio.file.FileSystems;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.nio.file.ProviderNotFoundException;
import java.security.CodeSource;
import java.util.Collection;
import java.util.Properties;
import java.util.stream.Collectors;

import org.junit.Before;
import org.junit.Ignore;
import org.junit.Test;
import org.junit.runner.RunWith;
import org.junit.runners.Parameterized;
import org.junit.runners.Parameterized.Parameter;
import org.junit.runners.Parameterized.Parameters;
import org.usfirst.frc.team2084.CMonster2016.Robot;

/**
 * @author Ben Wolsieffer
 */
@RunWith(Parameterized.class)
@Ignore
public class ParameterFileValidationTest {

    private static Path codePath;

    @Parameter
    public Path parameterPath;
    private Properties parameters;

    @Before
    public void loadParameters() throws IOException {
        InputStream stream = codePath.getFileSystem().provider().newInputStream(parameterPath.toAbsolutePath());
        parameters = new Properties();
        parameters.load(stream);
    }

    @Test
    public void checkParameterProperties() {
        parameters.entrySet().stream().forEach((p) -> {
            String key = (String) p.getKey();
            String val = (String) p.getValue();
            assertThat("Value must not be empty", val, not(isEmptyOrNullString()));
            assertThat("Key must not be empty", key, not(isEmptyOrNullString()));
        });
    }

    @Parameters(name = "Parameter file: {0}")
    public static Collection<Path> files() throws IOException {
        CodeSource src = Robot.class.getProtectionDomain().getCodeSource();
        Path path = null;
        try {
            path = Paths.get(src.getLocation().toURI());
        } catch (URISyntaxException e) {
        }
        try {
            FileSystem codeFileSystem =
                    FileSystems.newFileSystem(path, ParameterFileValidationTest.class.getClassLoader());
            codePath = codeFileSystem.getPath("/");
        } catch (ProviderNotFoundException | IOException ex) {
            codePath = path;
        }

        Path parameterPath = codePath.resolve("parameters");
        return Files.list(parameterPath).collect(Collectors.toList());
    }
}
