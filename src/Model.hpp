#pragma once
#include <halp/audio.hpp>
#include <halp/controls.hpp>
#include <halp/meta.hpp>
#include <halp/log.hpp>

#include <BRTLibrary.h>

#include <iostream>



#define SOFA_FILEPATH "hrtf.sofa"
constexpr int HRTFRESAMPLINGSTEP = 15;
#define ILD_NearFieldEffect_44100 "NearFieldCompensation_ILD_44100.sofa"
constexpr float SOURCE1_INITIAL_AZIMUTH = 3.14159265358979f / 2.0;
constexpr float SOURCE1_INITIAL_ELEVATION = 3.14159265358979f / 2.0;
constexpr float SOURCE1_INITIAL_DISTANCE = 0.1;
constexpr float UI_MIN_AZIMUTH = -90.0;
constexpr float UI_MAX_AZIMUTH = 90.0;
constexpr float UI_MIN_ELEVATION = -90.0;
constexpr float UI_MAX_ELEVATION = 90.0;
constexpr float UI_MIN_DISTANCE = 0.1;
constexpr float UI_MAX_DISTANCE = 2.0;

template <typename C>
  requires halp::has_logger<C>
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
  halp_meta(category, "Spatial")
  halp_meta(vendor, "University of Malaga")
  halp_meta(url, "https://www.diana.uma.es")
  halp_meta(c_name, "brt_listener_model")

  // halp_meta(channels, 2)

  struct
  {
    halp::fixed_audio_bus<"Input", float, 1> audio;
#if defined(AVND_PD)
    struct
        : halp::hslider_f32<
              "Source Azimuth (90, -90)",
              halp::range{
                  .min = UI_MIN_AZIMUTH,
                  .max = UI_MAX_AZIMUTH,
                  .init = (UI_MAX_AZIMUTH + UI_MIN_AZIMUTH) / 2.0}>
    {
      void update(BrtListenerModel& m) { 
          if(m.ready)
        {
          m.setSourceAzimuth(value);
          m.logger.info("Azimuth {}", value);
        }
      }
    } sAzimuth;
    struct
        : halp::hslider_f32<
              "Source Elevation (90, -90)",
              halp::range{
                  .min = UI_MIN_ELEVATION,
                  .max = UI_MAX_ELEVATION,
                  .init = (UI_MAX_ELEVATION + UI_MIN_ELEVATION) / 2.0}> 
  {
      void update(BrtListenerModel& m) {
          if (m.ready) m.setSourceElevation(value);
        }
  } sElevation;
  struct
      : halp::hslider_f32<
            "Source Distance (0.1 , 2) m",
            halp::range{
                .min = UI_MIN_DISTANCE,
                .max = UI_MAX_DISTANCE,
                .init = (UI_MAX_DISTANCE + UI_MIN_DISTANCE) / 2.0}>
  {
      void update(BrtListenerModel& m) { 
          if (m.ready) m.setSourceDistance(value); 
      }
  } sDistance;

#elif defined(AVND_VST3)
    // VST3 input parameters ranging 0 to 1
    halp::hslider_f32<"Source Azimuth (0 is 90,  1 is -90)",   halp::range{.min = 0., .max = 1., .init = 0}>  sAzimuth;
    halp::hslider_f32<"Source Elevation (0 is 90,  1 is -90)", halp::range{.min = 0., .max = 1., .init = 0}>  sElevation;
    halp::hslider_f32<"Source Distance (0 is 0.1m, 1 is 2m)",  halp::range{.min = 0., .max = 1., .init = 0}>  sDistance;
#endif 
    halp::toggle<"Enable Near Field", halp::toggle_setup{.init = true}> nearFieldEn; 
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
#if defined(AVND_VST3)
  /**
   * @brief Set the source distance in a value that goes from 0 (0.1) to 1 ()
   * 
   * @param vstValue 
   */
  void setVST3SourceDistance(float vstValue);

  /**
   * @brief Set the Source Azimuth in a value that goes from 0 (90) to 1 (-90 or 270 deg)
   * 
   * @param vstValue 
   */
  void setVST3SourceElevation(float vstValue);
 
  /**
   * @brief Set the Source Azimuth in a value that goes from 0 (90) to 1 (-90 or 270 deg)
   * 
   * @param vstValue 
   */
  void setVST3SourceAzimuth(float vstValue);
#endif

    /**
   * @brief Set the Source Distance in m (0.1 to 2m)
   * 
   * @param newDistance 
   */
  void setSourceDistance(float newDistance);


    /**
   * @brief Set the Source Elevation in radians (0 to 2*PI)
   * 
   * @param newElevation 
   */
  void setSourceElevation(float newElevation);

    /**
   * @brief Set the Source Azimuth in radians (0 to 2*PI)
   * 
   * @param newAzimuth 
   */
  void setSourceAzimuth(float newAzimuth);

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
