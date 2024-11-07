@ECHO OFF

set build_type=%1
if "%build_type%"=="" set build_type=STANDALONE

set hoslam_sdk_shared_libs=OFF

if /i "%build_type%"=="STANDALONE" (
rem echo standalone
set hoslam_sdk_shared_libs=OFF
) else if /i "%build_type%"=="SHARED" (
rem echo shared
set hoslam_sdk_shared_libs=ON
) else (
echo build_type %build_type% fail
goto fail
)
echo build_type=%build_type%

call mk_version.bat

SET PATH=%PATH%;d:\gcc-linaro-6.3.1-2017.02-i686-mingw32_arm-linux-gnueabihf\bin\
SET CURRENTDIR="%cd%"
SET CMAKE_FIND_ROOT_PATH=%CURRENTDIR%\buildroot

if not exist "build_windows\cartographer" mkdir build_windows\cartographer
cd build_windows/cartographer
cmake -G "Unix Makefiles" -DCMAKE_TOOLCHAIN_FILE=%CURRENTDIR%\toolchain\hoslam_pic.cmake -DCMAKE_PREFIX_PATH=%CURRENTDIR%\buildroot -DCMAKE_INSTALL_PREFIX=%CURRENTDIR%\buildroot %CURRENTDIR%\..\..\src\1.0\cartographer
rem keep this 'make install' for file-dependence
make install -j7
make -j4
make install -j7

if %errorlevel% neq 0 (
echo make cartographer failed.
goto fail
)

cd ..
if not exist "mini_eslam" mkdir mini_eslam
cd mini_eslam
cmake -G "Unix Makefiles" -DCMAKE_TOOLCHAIN_FILE=%CURRENTDIR%\toolchain\hoslam_pic.cmake -DCMAKE_PREFIX_PATH=%CURRENTDIR%\buildroot -DCMAKE_INSTALL_PREFIX=%CURRENTDIR%\buildroot %CURRENTDIR%\..\..\src\1.0\app\mini_eslam
make -j4 
make install -j7

if %errorlevel% neq 0 (
echo make mini_eslam failed.
goto fail
)

cd ..
if not exist "map_beautify" mkdir map_beautify
cd map_beautify
cmake -G "Unix Makefiles" -DCMAKE_TOOLCHAIN_FILE=%CURRENTDIR%\toolchain\hoslam_pic.cmake -DCMAKE_PREFIX_PATH=%CURRENTDIR%\buildroot -DCMAKE_INSTALL_PREFIX=%CURRENTDIR%\buildroot %CURRENTDIR%\..\..\src\1.0\app\map_beautify
make -j4
make install -j7

if %errorlevel% neq 0 (
echo make map_beautify failed.
goto fail
)

cd ..
if not exist "hoslam_sdk" mkdir hoslam_sdk
cd hoslam_sdk
cmake -G "Unix Makefiles" -DCMAKE_TOOLCHAIN_FILE=%CURRENTDIR%\toolchain\hoslam_pic.cmake -DCMAKE_PREFIX_PATH=%CURRENTDIR%\buildroot -DCMAKE_INSTALL_PREFIX=%CURRENTDIR%\buildroot -DBUILD_SHARED_LIBS=%hoslam_sdk_shared_libs% %CURRENTDIR%\..\..\src\1.0\app
make -j4
make install -j7

rem if %hoslam_sdk_shared_libs%==ON (
rem cd ..
rem if not exist "hoslam_hidden" mkdir hoslam_hidden
rem cd hoslam_hidden
rem cmake -G "Unix Makefiles" -DCMAKE_TOOLCHAIN_FILE=%CURRENTDIR%\toolchain\hoslam_pic.cmake -DCMAKE_PREFIX_PATH=%CURRENTDIR%\buildroot -DCMAKE_INSTALL_PREFIX=%CURRENTDIR%\buildroot -DUSE_SHARED_LIBS=%hoslam_sdk_shared_libs% %CURRENTDIR%\..\..\src\1.0\hoslam_hidden
rem make -j4
rem make install -j7
rem copy "%CURRENTDIR%\buildroot\lib\libhoslam_hidden.so" "%CURRENTDIR%\out\"
rem )

rem if %errorlevel% neq 0 (
rem echo make hoslam_sdk failed.
rem goto fail
rem )

if %hoslam_sdk_shared_libs%==OFF (
cd ..
if not exist demos mkdir demos
cd demos
cmake -G "Unix Makefiles" -DCMAKE_TOOLCHAIN_FILE=%CURRENTDIR%\toolchain\hoslam_pic.cmake -DCMAKE_PREFIX_PATH=%CURRENTDIR%\buildroot -DCMAKE_INSTALL_PREFIX=%CURRENTDIR%\buildroot -DUSE_SHARED_LIBS=%hoslam_sdk_shared_libs% %CURRENTDIR%\..\..\src\1.0\demos
make -j4
)

cd %CURRENTDIR%
if not exist "out" mkdir out
if %hoslam_sdk_shared_libs%==OFF (
copy "%CURRENTDIR%\build_windows\demos\demo_rplidar" "%CURRENTDIR%\out\demo_rplidar.debug"
copy "%CURRENTDIR%\buildroot\lib\libstdc++.so.6" "%CURRENTDIR%\out\libstdc++.so.6.debug"
arm-linux-gnueabihf-strip.exe --strip-unneeded %CURRENTDIR%\build_windows\demos\demo_rplidar -o %CURRENTDIR%\out\demo_rplidar
)

if not exist "%CURRENTDIR%\out\libstdc++.so.6" (
	arm-linux-gnueabihf-strip.exe --strip-unneeded %CURRENTDIR%\buildroot\lib\libstdc++.so.6 -o %CURRENTDIR%\out\libstdc++.so.6
)
if not exist "%CURRENTDIR%\out\libgfortran.so.3" (
	arm-linux-gnueabihf-strip.exe --strip-unneeded %CURRENTDIR%\buildroot\lib\libgfortran.so -o %CURRENTDIR%\out\libgfortran.so.3
)

if exist "%CURRENTDIR%\buildroot\lib\libcartographer.so" (
	copy "%CURRENTDIR%\buildroot\lib\libcartographer.so" "%CURRENTDIR%\out\libcartographer.so.debug"
	arm-linux-gnueabihf-strip.exe --strip-unneeded %CURRENTDIR%\buildroot\lib\libcartographer.so -o %CURRENTDIR%\out\libcartographer.so
)

if %hoslam_sdk_shared_libs%==ON (
if exist "%CURRENTDIR%\buildroot\lib\libhoslam_sdk.so" (
	copy "%CURRENTDIR%\buildroot\lib\libhoslam_sdk.so" "%CURRENTDIR%\out\libhoslam_sdk.so.debug"
	arm-linux-gnueabihf-strip.exe --strip-unneeded %CURRENTDIR%\buildroot\lib\libhoslam_sdk.so -o %CURRENTDIR%\out\libhoslam_sdk.so
)
)

cd %CURRENTDIR%

cd out
set t=%time%
set d=%date%
set xx=%t::=_%
set yy=%d:/=_%
set xx_prefix=%xx:~0,8%
set yy_prefix=%yy:~0,10%
set fname=demo_rplidar-%yy_prefix%_%xx_prefix%

copy "demo_rplidar" "%fname%"
copy libhoslam_sdk.so "libhoslam_sdk.so-%yy_prefix%_%xx_prefix%"


cd %CURRENTDIR%
echo "%date:~0,4%-%date:~5,2%-%date:~8,2% %time:~0,2%-%time:~3,2%-%time:~6,2% build finish."
pause
exit /b 0

:fail
cd %CURRENTDIR%
pause
exit /b 1
