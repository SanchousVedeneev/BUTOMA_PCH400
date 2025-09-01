@echo off
setlocal EnableDelayedExpansion

set "input_file=%1"
set "output_file=%2"

if not exist %output_file% (
    echo Creating file  %output_file%
)

set start_line=%3
set end_line=%4

@REM echo Input: %input_file%;
@REM echo Output: %output_file%;

@REM echo Start line: %start_line%
@REM echo End line: %end_line%

set "copy_content=false"

for /F "usebackq delims=" %%a in ("%input_file%") do (

    if "!copy_content!"=="true" (
        echo %%a>> "%output_file%"
    )
    if "%%a"==%start_line% (
        set "copy_content=true"
    )
    if "%%a"==%end_line% (
        set "copy_content=false"
    )
)

endlocal
