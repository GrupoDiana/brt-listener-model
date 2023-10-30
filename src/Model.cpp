#include "Model.hpp"

#include <filesystem>

Common::CVector3 BrtListenerModel::Spherical2Cartesians(float azimuth, float elevation, float radius) {

    float x = radius * cos(azimuth) * cos(elevation);
    float y = radius * sin(azimuth) * cos(elevation);
    float z = radius * sin(elevation);

    Common::CVector3 pos = listener->GetListenerTransform().GetPosition();

    pos.x = pos.x + x;
    pos.y = pos.y + y;
    pos.z = 0.0f;

    return pos;
}

void BrtListenerModel::operator()(int bufferSize)
{
  auto & in = inputs.audio;
  auto & out = outputs.audio; 

#if defined(AVND_VST3)
    float sAzimuth = inputs.sAzimuth.value;
    setVST3SourceAzimuth(sAzimuth);
#endif 
  CMonoBuffer<float> inputBuffer(&in[0][0], &in[0][bufferSize]);
  source->SetBuffer(inputBuffer);
  brtManager.ProcessAll();
  CMonoBuffer<float> left, right;
  listener->GetBuffers(left,right);
  for (int j = 0; j < bufferSize; j++) {
        out[0][j] = left[j];
  }
  for (int j = 0; j < bufferSize; j++) {
        out[1][j] = right[j];
      }
}

void BrtListenerModel::prepare(halp::setup info) {
  globalParameters.SetSampleRate(info.rate);
  globalParameters.SetBufferSize(info.frames);
  prepareBRT();
}

void BrtListenerModel::prepareBRT()
{
  // Setup Listener
  brtManager.BeginSetup();
  listener = brtManager.CreateListener<BRTListenerModel::CListenerHRTFbasedModel>(
      "listener1");
  brtManager.EndSetup();
  Common::CTransform listenerPosition = Common::CTransform(); // Setting listener in (0,0,0)
  listenerPosition.SetPosition(Common::CVector3(0, 0, 0));
  listener->SetListenerTransform(listenerPosition);

  // FIXME: For different platforms and bindings this will be a completely different
  // path, so not sure where to put sofa files. I put this line here to debug and copy files to where the debugger
  // is telling me the working directory is (chapuzza). 
  namespace fs = std::filesystem;
  volatile std::string pwd = fs::current_path().string();

  // Load hardcoded HRTF
  bool hrtfSofaLoaded = LoadSofaFile(SOFA_FILEPATH);

  // Set first HRTF for listener
  if (hrtfSofaLoaded) {
    listener->SetHRTF(HRTF_list[0]);
  }

  // LOAD NEARFIELD ILD coefficients 
    bool ildSofaLoaded = LoadILD(ILD_NearFieldEffect_44100);
    // Set to the listener
    if (ildSofaLoaded) {
        listener->SetILD(ILD_list[0]);
    }

  // Setup source
 
    brtManager.BeginSetup();
        source = brtManager.CreateSoundSource<BRTSourceModel::CSourceSimpleModel>("source1");    // Instatiate a BRT Sound Source
        listener->ConnectSoundSource(source);                                                   // Connect Source to the listener
    brtManager.EndSetup();          
    Common::CTransform sourcePose = Common::CTransform();                  
    sourcePose.SetPosition(Spherical2Cartesians(SOURCE1_INITIAL_AZIMUTH, SOURCE1_INITIAL_ELEVATION, SOURCE1_INITIAL_DISTANCE));
    source->SetSourceTransform(sourcePose);   

}




bool BrtListenerModel::LoadSofaFile(const std::string & _filePath) {
  
    std::shared_ptr<BRTServices::CHRTF> hrtf = std::make_shared<BRTServices::CHRTF>();

    int sampleRateInSOFAFile = sofaReader.GetSampleRateFromSofa(_filePath);
    if (sampleRateInSOFAFile == -1) {
        std::cout << ("Error loading HRTF Sofa file") << std::endl;
        return false;
    }
    if (globalParameters.GetSampleRate() != sampleRateInSOFAFile)
    {
        std::cout<<"The sample rate in HRTF SOFA file doesn't match the configuration." << std::endl;
        return false;
    }
    bool result = sofaReader.ReadHRTFFromSofa(_filePath, hrtf, HRTFRESAMPLINGSTEP);
    if (result) {
        std::cout << ("HRTF Sofa file loaded successfully.") << std::endl;
        HRTF_list.push_back(hrtf);
        return true;
    }
    else {
        std::cout << ("Error loading HRTF") << std::endl;
        return false;
    }
}

bool BrtListenerModel::LoadILD(const std::string & _ildFilePath) {
    std::shared_ptr<BRTServices::CILD> ild = std::make_shared<BRTServices::CILD>();
    
    
    int sampleRateInSOFAFile = sofaReader.GetSampleRateFromSofa(_ildFilePath);
    if (sampleRateInSOFAFile == -1) {
        std::cout << ("Error loading ILD Sofa file") << std::endl;
        return false;
    }
    if (globalParameters.GetSampleRate() != sampleRateInSOFAFile)
    {
        std::cout << "The sample rate in ILD SOFA file" << std::endl;
        return false;
    }
    
    bool result = sofaReader.ReadILDFromSofa(_ildFilePath, ild);
    if (result) {
        std::cout << "ILD Sofa file loaded successfully: " << std::endl;
        ILD_list.push_back(ild);
        return true;
    }
    else {
        std::cout << "Error loading HRTF" << std::endl;
        return false;
    }            
}

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
    return outgoing;
#endif
}

void BrtListenerModel::setVST3SourceAzimuth(float vstValue)
{
    // FIXME: Make sure vstValue is between 0 and 1. 
    float sourceAzimuth = 0.0;
    if(vstValue <= 0.5) sourceAzimuth = map(vstValue, 0, 0.5, PI_F / 2.0, 0);
    else sourceAzimuth = map(vstValue, 0.5, 1, 2.0 * PI_F, 3.0 * PI_F / 2.0);
    setSourceAzimuth(sourceAzimuth);
}

void BrtListenerModel::setSourceAzimuth(float newAzimuth)
{
    Common::CVector3 newPosition;
    newPosition = Spherical2Cartesians(newAzimuth, SOURCE1_INITIAL_ELEVATION, SOURCE1_INITIAL_DISTANCE);
    Common::CTransform newPose = source->GetCurrentSourceTransform();
    newPose.SetPosition(newPosition);
    source->SetSourceTransform(newPose);
}

