@echo off
pushd %~1
set WORLD_PATH="%CD%\%~2"
for /f "tokens=2 delims= " %%a in (".%WORLD_PATH:"=%.") do (
echo Webots does not support filepaths containing spaces. As such, you may not run this program in this directory.
pause
exit /b 1
)
start "" %WORLD_PATH%
popd
timeout /T 3 /nobreak
pushd %~3
call gradlew simulateJava -Dorg.gradle.java.home="C:\Users\Public\wpilib\2021\jdk"
popd
endlocal
@echo on