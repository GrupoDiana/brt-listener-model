#pragma once
#include <halp/audio.hpp>
#include <halp/controls.hpp>
#include <halp/meta.hpp>

#include <iostream>

class BrtListenerModel
{
public:
  halp_meta(name, "Brt Listener Model")
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
      void update(BrtListenerModel& m) { std::cerr << "okie " << value << "\n"; }
    } gain;
  } inputs;

  struct
  {
    halp::fixed_audio_bus<"Input", float, 1> audio;
  } outputs;

  void operator()(int N);

  // Defined in UI.hpp
  struct ui;
};
