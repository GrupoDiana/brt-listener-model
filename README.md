# BRT listener model

Work in progress with [Avendish](https://github.com/celtera/avendish) to port the [BRT Library](https://github.com/GrupoDiana/BRTLibrary) to PureData, VST3 and Max.
The portings are tested for Windows/MacOS. 

# Dependencies

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

## TODO

* Support for Max SDK
* Test on Linux
