@echo off

setlocal EnableDelayedExpansion

set "input_file=Makefile"
set "output_file=FromCubeMX.mk"

if exist %output_file% (
    FOR /F %%i IN ('DIR /B /O:D %input_file% %output_file%') DO call :Foo %%i
    goto :End
) else (
    goto :Copy
)

:Foo
set lastFile=%1
goto :eof
:End

if %lastFile%==%output_file% (
echo %output_file% no need to update
exit
) else (
echo del %output_file%
del %output_file%
)

:Copy

call .vscode/copyFromFileToFile.cmd %input_file% %output_file% "# debug build?" "#######################################"

call .vscode/copyFromFileToFile.cmd %input_file% %output_file% "# ASM sources" "#######################################"

call .vscode/copyFromFileToFile.cmd %input_file% %output_file% "# CFLAGS" "# C includes"

call .vscode/copyFromFileToFile.cmd %input_file% %output_file% "# LDFLAGS" "# libraries"


echo CUBE_C_FILES = \>> "%output_file%"
call .vscode/copyFromFileToFile.cmd %input_file% %output_file% "C_SOURCES =  \" "# ASM sources"

echo CUBE_C_INCLUDES = \>> "%output_file%"
call .vscode/copyFromFileToFile.cmd %input_file% %output_file% "C_INCLUDES =  \" "# compile gcc flags"





endlocal
