
cd .

if "%1"=="" ("D:\programs\matlab\bin\win64\gmake"  -f Pure_Pursuit.mk all) else ("D:\programs\matlab\bin\win64\gmake"  -f Pure_Pursuit.mk %1)
@if errorlevel 1 goto error_exit

exit /B 0

:error_exit
echo The make command returned an error of %errorlevel%
exit /B 1