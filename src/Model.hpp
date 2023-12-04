#pragma once
#include <BRTLibrary.h>
#include <halp/audio.hpp>
#include <halp/controls.hpp>
#include <halp/log.hpp>
#include <halp/meta.hpp>
#include <halp/layout.hpp>

#include <iostream>
#include <numbers>

#define SOFA_FILEPATH "hrtf.sofa"
constexpr int HRTFRESAMPLINGSTEP = 15;
#define ILD_NearFieldEffect_44100 "NearFieldCompensation_ILD_44100.sofa"
constexpr float SOURCE1_INITIAL_AZIMUTH = std::numbers::pi_v<float> / 2.0;
constexpr float SOURCE1_INITIAL_ELEVATION = 0.f;
constexpr float SOURCE1_INITIAL_DISTANCE = 0.1f;
constexpr float UI_MIN_AZIMUTH = -90.0;
constexpr float UI_MAX_AZIMUTH = 90.0;
constexpr float UI_MIN_ELEVATION = -90.0;
constexpr float UI_MAX_ELEVATION = 90.0;
constexpr float UI_MIN_DISTANCE = 0.1f;
constexpr float UI_MAX_DISTANCE = 2.0f;
constexpr float DEG_TO_RAD = std::numbers::pi_v<float> / 180.0;

template <typename C>
  requires halp::has_logger<C>
struct BrtListenerModel
{
    BrtListenerModel() noexcept{}
    
  [[no_unique_address]] typename C::logger_type logger;

#ifdef NDEBUG
  halp_meta(name, "Brt Listener Model")
  halp_meta(uuid, "32b00817-9c91-45b7-870e-fd3438a2c696")
#else
  halp_meta(name, "Brt Listener Model Debug")
  halp_meta(uuid, "410adb32-ed75-4908-8281-c5540b5870b0")
#endif
  halp_meta(category, "Spatial")
  halp_meta(vendor, "University of Malaga")
  halp_meta(url, "https://www.diana.uma.es")
  halp_meta(c_name, "brt_listener_model")

  /// <summary>
  /// prepare() sets this to true. 
  /// </summary>
  bool ready{false};

  struct
  {
    halp::fixed_audio_bus<"Input", float, 1> audio;
#if defined(AVND_PD)
    struct
        : halp::hslider_f32<
              "sAzimuth",
              halp::range{
                  .min = UI_MIN_AZIMUTH,
                  .max = UI_MAX_AZIMUTH,
                  .init = UI_MAX_AZIMUTH}>
    {
      void update(BrtListenerModel& m)
      {
        if(m.ready)
        {
          float fValue = value * DEG_TO_RAD;
          m.logger.trace("Azimuth: {}", fValue);
          m.setSourceAzimuth(fValue);
        }
      }
    } sAzimuth;
    struct
        : halp::hslider_f32<
              "sElevation",
              halp::range{
                  .min = UI_MIN_ELEVATION,
                  .max = UI_MAX_ELEVATION,
                  .init = (UI_MAX_ELEVATION + UI_MIN_ELEVATION) / 2.0}>
    {
      void update(BrtListenerModel& m)
      {
        if(m.ready)
          m.setSourceElevation(value * DEG_TO_RAD);
      }
    } sElevation;
    struct
        : halp::hslider_f32<
              "sDistance",
              halp::range{
                  .min = UI_MIN_DISTANCE,
                  .max = UI_MAX_DISTANCE,
                  .init = (UI_MAX_DISTANCE + UI_MIN_DISTANCE) / 2.0}>
    {
      void update(BrtListenerModel& m)
      {
        if(m.ready)
          m.setSourceDistance(value);
      }
    } sDistance;
      
    struct
      : halp::toggle<"enableNearField", halp::toggle_setup{.init = true}>
      {
          void update(BrtListenerModel &m) {
              if (m.ready) {
                  if (value) m.listener->EnableNearFieldEffect();
                  else m.listener->DisableNearFieldEffect();
                  m.logger.trace("Near field effect: {}", value);
              }
        }
      } enableNearField;

#elif defined(AVND_VST3)
    // VST3 input parameters ranging 0 to 1
    halp::hslider_f32<
        "Source Azimuth (0 is 90,  1 is -90)",
        halp::range{.min = 0., .max = 1., .init = 0}>
        sAzimuth;
    halp::hslider_f32<
        "Source Elevation (0 is 90,  1 is -90)",
        halp::range{.min = 0., .max = 1., .init = 0}>
        sElevation;
    halp::hslider_f32<
        "Source Distance (0 is 0.1m, 1 is 2m)",
        halp::range{.min = 0., .max = 1., .init = 0}>
        sDistance;
#endif

  } inputs;

  struct
  {
    halp::fixed_audio_bus<"Output", float, 2> audio;
  } outputs;

  /**
   * @brief The main process of the listener
   * 
   * @param N 
   */
  void operator()(int bufferSize)
  {
    if(ready)
    {
      auto& in = inputs.audio;
      auto& out = outputs.audio;

#if defined(AVND_VST3)
      float sAzimuth = inputs.sAzimuth.value;
      setVST3SourceAzimuth(sAzimuth);
      float sElevation = inputs.sElevation.value;
      setVST3SourceElevation(sElevation);
      float sDistance = inputs.sDistance.value;
      setVST3SourceDistance(sDistance);
#endif

      CMonoBuffer<float> inputBuffer(&in[0][0], &in[0][bufferSize]);
      source->SetBuffer(inputBuffer);
      brtManager.ProcessAll();
      CMonoBuffer<float> left, right;
      listener->GetBuffers(left, right);
      for(int j = 0; j < bufferSize; j++)
      {
        out[0][j] = left[j];
      }
      for(int j = 0; j < bufferSize; j++)
      {
        out[1][j] = right[j];
      }
    }
  }

  /**
   * @brief Prepare the listener model process
   * 
   * @param info provides all the audio state
   */
  void prepare(halp::setup info)
  {
    logger.trace("ready:{}", ready);
    if(!ready)
    {
      globalParameters.SetSampleRate(info.rate);
      globalParameters.SetBufferSize(info.frames);
      prepareBRT();
    }
    // else
    // {
    //   std::cerr << "Warning: in brt_listener_model: Ignoring attempt to reload resources"
    //             << std::endl;
    //   logger.warn(
    //       "WARNING: in brt_listener_model: Ignoring attempt to reload resources", "");
    // }

  }
#if(AVND_VST3)


  /**
   * @brief Set the source Azimuth in VST3 range. 
   * 
   * @param vstValue (0 is 90, 1 is -90)
   */
  void setVST3SourceAzimuth(float vstValue)
  {
    // FIXME: Make sure vstValue is between 0 and 1.
    float newAzimuth = 0.0;
    if(vstValue <= 0.5)
      newAzimuth = map(vstValue, 0, 0.5, PI_F / 2.0, 0);
    else
      newAzimuth = map(vstValue, 0.5, 1, 2.0 * PI_F, 3.0 * PI_F / 2.0);
    setSourceAzimuth(newAzimuth);
  }

  /**
   * @brief Set the source eleveation in VST3 range
   * 
   * @param vstValue (0 is 90,  1 is -90)
   */
  void setVST3SourceElevation(float vstValue)
  {
    // FIXME: Make sure vstValue is between 0 and 1.
    float newElevation = 0.0;
    if(vstValue <= 0.5)
      newElevation = map(vstValue, 0, 0.5, PI_F / 2.0, 0);
    else
      newElevation = map(vstValue, 0.5, 1, 2.0 * PI_F, 3.0 * PI_F / 2.0);
    setSourceElevation(newElevation);
  }

  /**
   * @brief Set the source distance to the listener in VST3 range
   * 
   * @param vstValue (0 is 0.1 m, 1 is 2 m)
   */
  void setVST3SourceDistance(float vstValue)
  {
    float newDistance = 0.0;
    newDistance = map(vstValue, 0., 1., UI_MIN_DISTANCE, UI_MAX_DISTANCE);
    setSourceDistance(newDistance);
  }

#endif

  /**
   * @brief Set the Source Azimuth 
   * 
   * @param newAzimuth in radians
   */
  void setSourceAzimuth(float newAzimuth)
  {
    Common::CVector3 newPosition;
    newPosition = Spherical2Cartesians(newAzimuth, sourceElevation, sourceDistance);
    Common::CTransform newPose = source->GetCurrentSourceTransform();
    newPose.SetPosition(newPosition);
    logger.trace("* Azimuth: {}", newAzimuth);
    logger.trace("* Source is at:{},{},{}", newPose.GetPosition().x, newPose.GetPosition().y, newPose.GetPosition().z);
    source->SetSourceTransform(newPose);
    sourceAzimuth = newAzimuth;
      }

  /**
   * @brief Set the Source Elevation 
   * 
   * @param newElevation in radians
   */
  void setSourceElevation(float newElevation)
  {
    Common::CVector3 newPosition;
    newPosition = Spherical2Cartesians(sourceAzimuth, newElevation, sourceDistance);
    Common::CTransform newPose = source->GetCurrentSourceTransform();
    newPose.SetPosition(newPosition);
    logger.trace("* Elevation: {}", newElevation);
    logger.trace(
        "* Source is at:{},{},{}", newPose.GetPosition().x, newPose.GetPosition().y,
        newPose.GetPosition().z);
    source->SetSourceTransform(newPose);
    sourceElevation = newElevation;
  }

  /**
   * @brief Set the Source Distance 
   * 
   * @param newDistance in meters
   */
  void setSourceDistance(float newDistance)
  {
    Common::CVector3 newPosition;
    newPosition = Spherical2Cartesians(sourceAzimuth, sourceElevation, newDistance);
    Common::CTransform newPose = source->GetCurrentSourceTransform();
    newPose.SetPosition(newPosition);
    logger.trace("* Distance: {}", newDistance);
    logger.trace(
        "* Source is at:{},{},{}", newPose.GetPosition().x, newPose.GetPosition().y,
        newPose.GetPosition().z);
    source->SetSourceTransform(newPose);
    sourceDistance = newDistance;
  }

  // Defined in UI.hpp
  struct ui
  {
    // If your compiler is recent enough:
    // using enum halp::colors;
    // using enum halp::layouts;
    halp_meta(layout, halp::layouts::vbox)
    halp_meta(background, halp::colors::mid)

    halp::label header{"Hello !"};

    struct
    {
      halp_meta(layout, halp::layouts::hbox)
      halp_meta(background, halp::colors::dark)
      //halp::item<&ins::gain> widget;
    } widgets;
  };

  /**
   * @brief Setup the BRT Library
   * 
   */
  void prepareBRT()
  {
    // Setup Listener
    brtManager.BeginSetup();
    listener = brtManager.CreateListener<BRTListenerModel::CListenerHRTFbasedModel>(
        "listener1");
    brtManager.EndSetup();
    Common::CTransform listenerPosition
        = Common::CTransform(); // Setting listener in (0,0,0)
    listenerPosition.SetPosition(Common::CVector3(0, 0, 0));
    listener->SetListenerTransform(listenerPosition);

    // FIXME: For different platforms and bindings this will be a completely different
    // Will designing how to approach this, I put this line here to debug and copy files to where the debugger
    // is telling me the working directory is, and then just copy the files there.  (chapuzza).
    namespace fs = std::filesystem;
    volatile std::string pwd = fs::current_path().string();
    fs::path sofaFilePath;
    fs::path ildFilePath;

#if(defined(AVND_VST3) || defined(AVND_PD))
#ifdef _WIN32
    //char* appdata = std::getenv("APPDATA");
    auto resourcePath = fs::path{std::getenv("APPDATA")};
#elif __APPLE__
    auto resourcePath = fs::path("/Library/Application Support");
#endif
    resourcePath.append("es.uma.3ddiana.brt").append("Resources");
    if(fs::exists(resourcePath))
    {
      sofaFilePath = resourcePath / "HRTF";
      ildFilePath = resourcePath / "ILD";
    }
#endif

    sofaFilePath.append(SOFA_FILEPATH);
    volatile bool sofaExists = fs::exists(sofaFilePath);
    ildFilePath.append(ILD_NearFieldEffect_44100);

    // Load hardcoded HRTF
    bool hrtfSofaLoaded = LoadSofaFile(sofaFilePath.string());

    // Set first HRTF for listener
    if(hrtfSofaLoaded)
    {
      listener->SetHRTF(HRTF_list[0]);
    }

    // LOAD NEARFIELD ILD coefficients
    bool ildSofaLoaded = LoadILD(ildFilePath.string());
    // Set to the listener
    if(ildSofaLoaded)
    {
      listener->SetILD(ILD_list[0]);
    }

    // Setup source

    brtManager.BeginSetup();
    source = brtManager.CreateSoundSource<BRTSourceModel::CSourceSimpleModel>(
        "source1");                       // Instatiate a BRT Sound Source
    listener->ConnectSoundSource(source); // Connect Source to the listener
    brtManager.EndSetup();
    Common::CTransform sourcePose = Common::CTransform();
    sourceAzimuth = SOURCE1_INITIAL_AZIMUTH;
    sourceElevation = SOURCE1_INITIAL_ELEVATION;
    sourceDistance = SOURCE1_INITIAL_DISTANCE;
    sourcePose.SetPosition(Spherical2Cartesians(
        SOURCE1_INITIAL_AZIMUTH, SOURCE1_INITIAL_ELEVATION, SOURCE1_INITIAL_DISTANCE));
    source->SetSourceTransform(sourcePose);
    ready = true;
  }

  /**
   * @brief Load SOFA HRTF File
   * 
   * @param _filePath 
   * @return true 
   * @return false 
   */
  bool LoadSofaFile(const std::string& _filePath)
  {

    std::shared_ptr<BRTServices::CHRTF> hrtf = std::make_shared<BRTServices::CHRTF>();

    int sampleRateInSOFAFile = sofaReader.GetSampleRateFromSofa(_filePath);
    if(sampleRateInSOFAFile == -1)
    {
      std::cout << ("Error loading HRTF Sofa file") << std::endl;
      return false;
    }
    if(globalParameters.GetSampleRate() != sampleRateInSOFAFile)
    {
      std::cout << "The sample rate (" << sampleRateInSOFAFile
                << ") in the HRTF SOFA file " << _filePath
                << "doesn't match the audio configuration ("
                << globalParameters.GetSampleRate() << ")." << std::endl;
      return false;
    }
    bool result = sofaReader.ReadHRTFFromSofa(_filePath, hrtf, HRTFRESAMPLINGSTEP);
    if(result)
    {
      std::cout << ("HRTF Sofa file loaded successfully.") << std::endl;
      HRTF_list.push_back(hrtf);
      return true;
    }
    else
    {
      std::cout << ("Error loading HRTF") << std::endl;
      return false;
    }
  }

  /**
   * @brief Load ILD Coefficientes
   * 
   * @param _filePath 
   * @return true 
   * @return false 
   */
  bool LoadILD(const std::string& _ildFilePath)
  {
    std::shared_ptr<BRTServices::CILD> ild = std::make_shared<BRTServices::CILD>();

    int sampleRateInSOFAFile = sofaReader.GetSampleRateFromSofa(_ildFilePath);
    if(sampleRateInSOFAFile == -1)
    {
      std::cout << ("Error loading ILD Sofa file") << std::endl;
      return false;
    }
    if(globalParameters.GetSampleRate() != sampleRateInSOFAFile)
    {
      std::cout << "The sample rate in ILD SOFA file" << std::endl;
      return false;
    }

    bool result = sofaReader.ReadILDFromSofa(_ildFilePath, ild);
    if(result)
    {
      std::cout << "ILD Sofa file loaded successfully: " << std::endl;
      ILD_list.push_back(ild);
      return true;
    }
    else
    {
      std::cout << "Error loading HRTF" << std::endl;
      return false;
    }
  }

  Common::CVector3 Spherical2Cartesians(float azimuth, float elevation, float radius)
  {

    Common::CVector3 globalPos = listener->GetListenerTransform().GetPosition();

    float x = radius * cos(azimuth) * cos(elevation);
    float y = radius * sin(azimuth) * cos(elevation);
    float z = radius * sin(elevation);

    globalPos.x += x;
    globalPos.y += y;
    globalPos.z += z;

    return globalPos;
  }

  // BRT vars

  Common::CGlobalParameters globalParameters;
  BRTBase::CBRTManager brtManager;
  std::shared_ptr<BRTListenerModel::CListenerHRTFbasedModel> listener;
  std::shared_ptr<BRTSourceModel::CSourceSimpleModel> source;
  float sourceAzimuth{SOURCE1_INITIAL_AZIMUTH};
  float sourceElevation{SOURCE1_INITIAL_ELEVATION};
  float sourceDistance{SOURCE1_INITIAL_DISTANCE};
  BRTReaders::CSOFAReader sofaReader;
  std::vector<std::shared_ptr<BRTServices::CHRTF>> HRTF_list;
  std::vector<std::shared_ptr<BRTServices::CILD>> ILD_list;
};

/*
  This function is based on PApplet::map which is part of the Processing project - http://processing.org

  Copyright (c) 2012-15 The Processing Foundation
  Copyright (c) 2004-12 Ben Fry and Casey Reas
  Copyright (c) 2001-04 Massachusetts Institute of Technology

  distributed under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation, version 2.1.
*/
float map(float value, float start1, float stop1, float start2, float stop2)
{
  float outgoing = start2 + (stop2 - start2) * ((value - start1) / (stop1 - start1));
#ifndef NDEBUG
  if(std::isnan(outgoing))
  {
    volatile int errorNan;
  }
  else if(std::isinf(outgoing))
  {
    volatile int errorInf;
  }
#endif
  return outgoing;
}
