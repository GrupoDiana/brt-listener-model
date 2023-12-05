#!/bin/bash
/usr/bin/osascript -e 'display dialog "This script will copy all the BRT externals to /Library/Pd/. For this you need to run the script as administrator (for example with sudo). It will also copy all the BRT resource files to /Library/Application Support/es.uma.3ddiana.brt/. Note that if you move around these files the VST3 BRT plugin will not work (and fail silently)" '

echo "Installing resources..."
mkdir -p /Library/Application\ Support/es.uma.3ddiana.brt/Resources
cp -vr ../resources/* /Library/Application\ Support/es.uma.3ddiana.brt/Resources/
echo "finsihed"

echo "Installing externals..."
mkdir -p /Library/Pd/
cp -vr ../pd/brt_listener_model.pd_darwin /Library/Pd/
echo "finished"