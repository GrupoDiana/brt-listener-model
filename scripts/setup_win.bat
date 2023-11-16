
mshta "about:<script>alert('This script will copy all the BRT plugins to C:\\Program Files\\Common Files\\VST3\\. For this you need to run the script as administrator. It will also copy all the BRT resource files to your %%APPDATA%% directory. Note that if you move around these files the VST3 BRT plugin will not work (and fail silently).');close()</script>"

echo %appdata%

pushd %~dp0
xcopy /s ..\resources\  "%appdata%\es.uma.3ddiana.brt\Resources\"
xcopy /s ..\vst3plugins\ "%commonProgramFiles%\VST3\"
popd

pause
