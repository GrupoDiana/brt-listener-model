#!/bin/bash
/usr/bin/osascript -e 'display dialog "This script will copy all the BRT plugins to /Library/Audio/Plug-ins/VST3. For this you need to run the script as administrator (for example with sudo). It will also copy all the BRT resource files to /Library/Application Support/es.uma.3ddiana.brt/. Note that if you move around these files the VST3 BRT plugin will not work (and fail silently)" '

echo "Installing resources..."
mkdir -p /Library/Application\ Support/es.uma.3ddiana.brt/Resources
cp -r ../resources/* /Library/Application\ Support/es.uma.3ddiana.brt/Resources/
echo "finsihed"

echo "\nInstalling plug-ins..."
cp -r ../vst3plugins/BrtListenerModel_vst3.vst3 /Library/Audio/Plug-ins/VST3/
echo "finished"