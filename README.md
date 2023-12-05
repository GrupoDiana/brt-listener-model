# BRT listener model

Work in progress with [Avendish](https://github.com/celtera/avendish) to port the [BRT Library](https://github.com/GrupoDiana/BRTLibrary) to PureData, VST3 and Max.
The portings are tested for Windows/MacOS. 

## Dependencies

* The BRT Library is automatically fetched from github, but not its dependencies, which are:
   * Nlohmann json: [Get it there](https://github.com/nlohmann/json) and install.
   * Libmysofa: [Get it from our fork here](https://github.com/GrupoDiana/libmysofa/tree/cmake) and install.  
* For the PureData, [Get it there](https://github.com/pure-data/pure-data), build it, and pass to cmake:

```cmake
-DCMAKE_PREFIX_PATH=path/to/the/folder/containing/m_pd.h -DBRT_CREATE_PD=ON
```
* For VST3, [Get it there](https://github.com/steinbergmedia/vst3sdk) and pass to cmake:

```cpp
-DVST3_SDK_ROOT=path/to/vst3 -DBRT_CREATE_VST3=ON
```

## Examples 

There are two examples for PureData and REAPER in the examples folder. 
![test-brt-listener-model.pd](https://github.com/GrupoDiana/brt-listener-model/assets/1093084/34008568-2c67-459c-bd25-c19697f05377)

![test-brt-listener-44100.RPP](https://github.com/GrupoDiana/brt-listener-model/assets/1093084/bef84219-41ae-4c28-b453-00713b9a21f0)




## TODO

* Support for Max SDK
* Test on Linux

## Credits

This software is being developed by a team coordinated by 
-	[Arcadio Reyes-Lecuona](https://github.com/areyesl) ([University of Malaga](https://www.uma.es/)). Contact: areyes@uma.es  

The current members of the development team are (in alphabetical order):
- [Maria Cuevas-Rodriguez](https://github.com/mariacuevas) ([University of Malaga](https://www.uma.es/))
- [Daniel Gonzalez-Toledo](https://github.com/dgonzalezt) ([University of Malaga](https://www.uma.es/))
- [Luis Molina-Tanco](https://github.com/lmtanco) ([University of Malaga](https://www.uma.es/))
- [Francisco Morales-Benitez](https://github.com//FranMoraUma) ([University of Malaga](https://www.uma.es/))


## Copyright and License

Copyright(c) University of Malaga – 2023 

[Avendish](https://github.com/celtera/avendish) is is licensed under GPLv3. 

The [BRT Library](https://github.com/GrupoDiana/BRTLibrary) includes pieces of code from the 3DTI AudioToolkit, shared under GPLv3 license and copyright (c) by University of Málaga (contact: areyes@uma.es) and Imperial College London (contact: l.picinali@imperial.ac.uk). 

This program is free software, you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
