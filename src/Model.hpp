#pragma once
#include <halp/audio.hpp>
#include <halp/controls.hpp>
#include <halp/meta.hpp>

#include <BRTLibrary.h>

#include <iostream>

#define SOFA_FILEPATH "hrtf.sofa"
constexpr int HRTFRESAMPLINGSTEP = 15;
#define ILD_NearFieldEffect_44100 "NearFieldCompensation_ILD_44100.sofa"
constexpr float SOURCE1_INITIAL_AZIMUTH = 3.14159265358979f / 2.0;
constexpr float SOURCE1_INITIAL_ELEVATION = 3.14159265358979f / 2.0;
constexpr float SOURCE1_INITIAL_DISTANCE = 0.1;

class BrtListenerModel
{
public:
#ifdef NDEBUG
  halp_meta(name, "Brt Listener Model")
  halp_meta(uuid, "32b00817-9c91-45b7-870e-fd3438a2c696")
#else
  halp_meta(name, "Brt Listener Model Debug")
    halp_meta(uuid, "410adb32-ed75-4908-8281-c5540b5870b0")

#endif
  halp_meta(c_name, "brt_listener_model")

  // halp_meta(channels, 2)

  struct
  {
    halp::fixed_audio_bus<"Input", float, 1> audio;
    struct : halp::hslider_f32<"Source Azimuth (0 is 90,  1 is -90)", halp::range{.min = 0., .max = 1., .init = 0}>
    {
      void update(BrtListenerModel& m) { 
          std::cerr << "okie " << value << "\n"; 
      }
    } sAzimuth;
    struct
        : halp::hslider_f32<
              "Source Elevation (0 is 90,  1 is -90)", halp::range{.min = 0., .max = 1., .init = 0.5}>
    {
      void update(BrtListenerModel& m) { std::cerr << "okie " << value << "\n"; }
    } sElevation;
    struct : halp::hslider_f32<"Source Distance (0 is 0.1m, 1 is 2m)", halp::range{.min = 0., .max=1., .init = 0.5}>
    {
      void update(BrtListenerModel& m) {std::cerr << "okie " << value << "\n"; }
    } sDistance;
  } inputs;

  struct
  {
    halp::fixed_audio_bus<"Input", float, 2> audio;
  } outputs;

  /**
   * @brief The main process of the listener
   * 
   * @param N 
   */
  void operator()(int N);

  /**
   * @brief Prepare the listener model process
   * 
   * @param info provides all the audio state
   */
  void prepare(halp::setup info);

  /**
   * @brief Set the Source Distance in m (0.1 to 2m)
   * 
   * @param newDistance 
   */
  void setSourceDistance(float newDistance);

  /**
   * @brief Set the source distance in a value that goes from 0 (0.1) to 1 ()
   * 
   * @param vstValue 
   */
  void setVST3SourceDistance(float vstValue);

  /**
   * @brief Set the Source Elevation in radians (0 to 2*PI)
   * 
   * @param newElevation 
   */
  void setSourceElevation(float newElevation);

  /**
   * @brief Set the Source Azimuth in a value that goes from 0 (90) to 1 (-90 or 270 deg)
   * 
   * @param vstValue 
   */
  void setVST3SourceElevation(float vstValue);

    /**
   * @brief Set the Source Azimuth in radians (0 to 2*PI)
   * 
   * @param newAzimuth 
   */
  void setSourceAzimuth(float newAzimuth);

  /**
   * @brief Set the Source Azimuth in a value that goes from 0 (90) to 1 (-90 or 270 deg)
   * 
   * @param vstValue 
   */
  void setVST3SourceAzimuth(float vstValue);

  // Defined in UI.hpp
  struct ui;


private:  

  /**
   * @brief Setup the BRT Library
   * 
   */
  void prepareBRT();

  /**
   * @brief Load SOFA HRTF File
   * 
   * @param _filePath 
   * @return true 
   * @return false 
   */
  bool LoadSofaFile(const std::string & _filePath);

  /**
   * @brief Load ILD Coefficientes
   * 
   * @param _filePath 
   * @return true 
   * @return false 
   */
  bool LoadILD(const std::string& _filePath);



  Common::CVector3 Spherical2Cartesians(float azimuth, float elevation, float radius);

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
