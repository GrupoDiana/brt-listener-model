
mshta "about:<script>alert('This script will copy all the BRT externals to C:\\Program Files(x86)\\pd\\extra\\brt. For this you need to run the script as administrator. It will also copy all the BRT resource files to your %%APPDATA%% directory. Note that if you move around these files the BRT external will not work (and fail silently).');close()</script>"

echo %appdata%

pushd %~dp0
xcopy /s ..\resources\  "%appdata%\es.uma.3ddiana.brt\Resources\"
xcopy /s ..\pd\ "%ProgramFiles(x86)%\pd\extra\brt\"
popd

pause
