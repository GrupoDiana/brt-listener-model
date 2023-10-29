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