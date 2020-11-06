package org.carlmontrobotics.gradle

import groovy.transform.CompileStatic
import org.gradle.api.Action
import org.gradle.api.Plugin
import org.gradle.api.Project
import org.gradle.api.DefaultTask
import org.gradle.api.artifacts.Dependency
import org.gradle.api.tasks.compile.JavaCompile
import org.gradle.api.tasks.testing.Test
import org.gradle.internal.os.OperatingSystem
import org.gradle.jvm.tasks.Jar
import org.gradle.api.tasks.StopActionException
import org.slf4j.LoggerFactory

@CompileStatic
class WebotsPlugin implements Plugin<Project> {

    @Override
    void apply(Project project) {
//        project.plugins.withId("edu.wpi.first.GradleRIO") {
            def log = LoggerFactory.getLogger('webots-plugin-logger')
            def os = OperatingSystem.current()
            String envDelim = os.isWindows() ? ";" : ":"
            def webots_home = ""
            List<String> dirs_to_check = new ArrayList<String>()

            def webots_home_env = System.getenv("WEBOTS_HOME")
            if (webots_home_env) {
                dirs_to_check.add(webots_home_env)
            }

            if (dirs_to_check.size() == 0) {
                log.info "WEBOTS_HOME environment variable is not set, so trying to set it automatically."
                if (os.isLinux()) {
                    dirs_to_check.add "/usr/local/webots"
                } else if (os.isMacOsX()) {
                    dirs_to_check.add "/Applications/Webots.app"
                } else if (os.isWindows()) {
                    dirs_to_check.add System.getenv("USER_HOME") + "/Webots"
                    dirs_to_check.add System.getenv("LOCALAPPDATA") + "/Programs/Webots"
                }
            }
            for (d in dirs_to_check) {
                log.info "Checking '" + d + "'"

                def f = new File(d)
                if (f.exists()) {
                    log.info "Found it!"
                    webots_home = d
                    break
                }
            }

            def ldpath = webots_home + "/lib/controller"
            project.repositories.flatDir([ dirs: ldpath+"/java"])
            def controllerJar = project.files(ldpath+"/java/Controller.jar")
            project.dependencies.add("compile", controllerJar)

            def desktopLibs = project.configurations.getByName('nativeDesktopLib')
            def libs = project.files(project.fileTree(ldpath).getFiles())
            log.info "libs = " + libs.getFiles()
            project.dependencies.add("nativeDesktopLib", libs)

            project.tasks.withType(JavaCompile).configureEach { JavaCompile t ->
                if (webots_home == "") {
                    throw new StopActionException("Unable to find Webots! Set the WEBOTS_HOME environment variable to the path of your Webots installation")
                }
                t.classpath += project.files(webots_home + "/lib/controller/java/Controller.jar")
                return t
            }
            project.tasks.withType(DefaultTask).configureEach { DefaultTask t ->
                t.doFirst {
                    // def controllerJar = project.files(webots_home + "/lib/controller/java/Controller.jar")
                    // def cp = System.getenv("CLASSPATH")
                    // if (!cp) {
                    //     cp = ""
                    // }
                    // if (cp.indexOf(controllerJar) == -1)
                    //     cp = cp + envDelim + controllerJar 

                    // println "cp = " + cp
                    // System.setenv("CLASSPATH", cp)
                    // def nativeLibs = project.configurations.getByName('nativeDesktopLib')
                    // println "nativeDesktopLibs = " + nativeLibs.dependencies
                    // nativeLibs.dependencies
                    //     .matching { Dependency dep -> dep != null && nativeLibs.files(dep).size() > 0 }
                    //     .all { Dependency dep ->
                    //         def fc = project.files(nativeLibs.files(dep).toArray())
                    //         println "nativeDep = " + fc.getFiles()
                    //     }
                }
                return t
            }
            project.tasks.withType(Test).configureEach { Test t ->
                t.doFirst {
                    if (webots_home == "") {
                        throw new StopActionException("Unable to find Webots! Set the WEBOTS_HOME environment variable to the path of your Webots installation")
                    }
                    def env = t.getEnvironment()

                    env["WEBOTS_HOME"] = webots_home

                    if (os.isLinux()) {
                        env["LD_LIBRARY_PATH"] = (String)env["LD_LIBRARY_PATH"] + envDelim + ldpath
                    } else if (os.isMacOsX()) {
                        env["DYLD_LIBRARY_PATH"] = (String)env["DYLD_LIBRARY_PATH"] + envDelim + ldpath
                    } else if (os.isWindows()) {
                        env["PATH"] = (String)env["PATH"] + envDelim + ldpath
                        env["PATH"] = (String)env["PATH"] + envDelim + webots_home + "/msys64/mingw64/bin"
                        env["PATH"] = (String)env["PATH"] + envDelim + webots_home + "/msys64/mingw64/bin/cpp"
                    }

                    t.environment(env)

                    def jlp = ldpath
                    if (t.systemProperties.containsKey('java.library.path'))
                        jlp = (String)t.systemProperties.get('java.library.path') + envDelim + ldpath
                    t.systemProperties.put('java.library.path', jlp)
                    log.info "Added " + ldpath + " to java.library.path"
                }
                return t
            }
//        }
    }
}
