#pragma once
#include <halp/audio.hpp>
#include <halp/controls.hpp>
#include <halp/meta.hpp>

#include <BRTLibrary.h>

#include <iostream>

#define SOFA_FILEPATH "hrtf.sofa"
#define HRTFRESAMPLINGSTEP 15
#define ILD_NearFieldEffect_44100 "NearFieldCompensation_ILD_44100.sofa"
#define SOURCE1_INITIAL_AZIMUTH     90
#define SOURCE1_INITIAL_ELEVATION   0
#define SOURCE1_INITIAL_DISTANCE    2

class BrtListenerModel
{
public:
#ifdef NDEBUG
  halp_meta(name, "Brt Listener Model")
#else
  halp_meta(name, "Brt Listener Model Debug")
#endif
  halp_meta(c_name, "brt_listener_model")

  // CHANGE THIS !!
  // - On linux: uuidgen | xargs printf | xclip -selection clipboard
  //   will copy one on the clipboard
  // - uuidgen exists on Mac and Windows too
  halp_meta(uuid, "32b00817-9c91-45b7-870e-fd3438a2c696")
  // halp_meta(channels, 2)

  struct
  {
    halp::fixed_audio_bus<"Input", float, 1> audio;
    struct : halp::hslider_f32<"Gain", halp::range{.min = 0., .max = 1., .init = 0.5}>
    {
      void update(BrtListenerModel& m) { 
          std::cerr << "okie " << value << "\n"; 
      }
    } gain;
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
  bool LoadILD(const std::string & _filePath);

  Common::CVector3 Spherical2Cartesians(float azimuth, float elevation, float radius);

  // BRT vars	
	Common::CGlobalParameters globalParameters;
	BRTBase::CBRTManager brtManager;
	std::shared_ptr<BRTListenerModel::CListenerHRTFbasedModel> listener;
  std::shared_ptr<BRTSourceModel::CSourceSimpleModel> source;
  BRTReaders::CSOFAReader sofaReader;
  std::vector<std::shared_ptr<BRTServices::CHRTF>> HRTF_list;  
  std::vector<std::shared_ptr<BRTServices::CILD>> ILD_list;    	
};
