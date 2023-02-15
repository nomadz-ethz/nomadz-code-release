/**
 * @file TfLiteWhistleRecognizer.cpp
 *
 * Implementation of module that identifies the sound of a whistle
 *
 * This file is subject to the terms of MIT License.
 * A copy of this license is included in LICENSE.txt.
 * (c) 2022 NomadZ team
 */

#include "TfLiteWhistleRecognizer.h"
#include "Core/Settings.h"
#include "Core/Debugging/DebugDrawings.h"

#include <limits>

#include "Core/System/Thread.h"
#include "Core/System/File.h"
#include "Representations/Infrastructure/TeamMateData.h"

#include "tensorflow/lite/kernels/register.h"
#include "tensorflow/lite/optional_debug_tools.h"

// Prints additional details about the network.
// #define WHISTLE_DEBUG

TfLiteWhistleRecognizer::TfLiteWhistleRecognizer() {
  for (int i = 0; i < WHISTLE_BUFF_LEN; ++i) {
    inputChannel0.add(0);
    inputChannel1.add(0);
  }
  cmpCnt = 0;
  lastGameState = STATE_INITIAL;
  lastTimeWhistleDetectedInBothChannels = 0;

  // Allocate buffers
  whistleInput8kHz = (double*)calloc(WHISTLE_BUFF_LEN, sizeof(double));
  whistleMFCCs = (double*)calloc(WHISTLE_NUM_FRAMES * WHISTLE_NUM_MFCCS, sizeof(double));

  // Load the model and initialize interpreter
  auto absolutePath = std::string(File::getBHDir()) + "/" + pathToModel;
  model_ = tflite::FlatBufferModel::BuildFromFile(absolutePath.c_str());
  ASSERT(model_ != nullptr);

  tflite::ops::builtin::BuiltinOpResolver resolver;
  tflite::InterpreterBuilder builder(*model_, resolver);
  builder(&interpreter_);
  VERIFY(interpreter_->AllocateTensors() == kTfLiteOk);

#ifdef WHISTLE_DEBUG
  OUTPUT_TEXT("TfLiteWhistleRecognizer: Loaded model and initialized interpreter!");
  EXECUTE_ONLY_IN_DEBUG(tflite::PrintInterpreterState(interpreter_.get()););
#endif

  // Fetch input shape and check dimensions
  VERIFY(interpreter_->inputs().size() == 1 && interpreter_->outputs().size() == 1);
  auto input = interpreter_->input_tensor(0);
  auto output = interpreter_->output_tensor(0);
  VERIFY(input->dims->size == 4 && input->dims->data[0] == 1 && input->dims->data[1] == WHISTLE_NUM_FRAMES &&
         input->dims->data[2] == WHISTLE_NUM_MFCCS && input->dims->data[3] == 1);
  VERIFY(output->dims->size == 2 && output->dims->data[0] == 1 && output->dims->data[1] == 2);

  input_scale = input->params.scale;
  input_zero_point = input->params.zero_point;
  output_scale = output->params.scale;
  output_zero_point = output->params.zero_point;
}

TfLiteWhistleRecognizer::~TfLiteWhistleRecognizer() {
  free(whistleInput8kHz);
  free(whistleMFCCs);
}

void TfLiteWhistleRecognizer::update(Whistle& whistle) {
  whistle.whistleDetected = false;

  if (theFallDownState.state != FallDownState::upright) {
    firstTimeUpright = 0;
  } else if (firstTimeUpright == 0) {
    firstTimeUpright = theFrameInfo.time;
  }

  // If two or more other robots hear it, it is a whistle
  int whistleHeard = 0;
  for (size_t i = 1; i <= 5; i++) {
    if (theTeamMateData.isFullyActive[i] && theTeamMateData.whistle[i].whistleDetected) {
      whistleHeard++;
    }
  }
  if (whistleHeard >= 2) {
    whistle.lastTimeWhistleDetected = theFrameInfo.time;
    whistle.confidenceOfLastWhistleDetection = 50;
    whistle.whistleDetected = true;
  }
  // Only listen to the whistle in set state and clear
  // the buffers when entering a set state:
  if (!whistle.whistleDetected) {
    lastGameState = theGameInfo.state;
    if (theGameInfo.state != STATE_SET) {
      return;
    }
    if (theFallDownState.state != FallDownState::upright || theFrameInfo.getTimeSince(firstTimeUpright) < 2000) {
      return;
    }

    if ((lastGameState != STATE_SET && theGameInfo.state == STATE_SET) ||
        (theRobotInfo.penalty == PENALTY_SPL_ILLEGAL_MOTION_IN_SET)) {
      // Clear audio channels
      for (int i = 0; i < WHISTLE_BUFF_LEN; ++i) {
        inputChannel0.add(0);
        inputChannel1.add(0);
      }
      cmpCnt = 0;
    }

    // Check input data:
    if (theAudioData.channels != 2) {
      OUTPUT_TEXT("Wrong number of channels! TfLiteWhistleRecognizer expects 2 channels, but AudioData has "
                  << theAudioData.channels << "!");
    }
    if (theAudioData.samples.size() == 0) {
      return;
    } else {
      whistle.lastTimeOfIncomingSound = theFrameInfo.time;
    }

    // Add incoming audio data to the two buffers
    unsigned int i = 0;
    while (i < theAudioData.samples.size()) {
      cmpCnt++;
      const short sample0 = theAudioData.samples[i++];
      const short sample1 = theAudioData.samples[i++];
      inputChannel0.add(sample0);
      inputChannel1.add(sample1);
    }

    // Recognize the whistle
    double probChannel0 = 0.0;
    double probChannel1 = 0.0;
    float currentVolume = 0.0;
    if (inputChannel0.full() && cmpCnt >= WHISTLE_OVERLAP) {
      cmpCnt = 0;
      currentVolume = computeCurrentVolume();

      const bool vol = currentVolume > volumeThreshold;
      const bool w0 = detectWhistle(inputChannel0, probChannel0, whistle);
      const bool w1 = detectWhistle(inputChannel1, probChannel1, whistle);
      float maxProb = 0.f;
      if (probChannel0 >= maxProb) {
        maxProb = probChannel0;
      }
      if (probChannel1 >= maxProb) {
        maxProb = probChannel1;
      }
      if (vol) {
        OUTPUT_TEXT("Current volume passes whistle threshold: " << currentVolume);
      }

      // Best case: Everything is fine!
      if (vol && w0 && w1 && !theDamageConfigurationHead.audioChannel0Defect &&
          !theDamageConfigurationHead.audioChannel1Defect) {
        lastTimeWhistleDetectedInBothChannels = theFrameInfo.time;
        whistle.lastTimeWhistleDetected = theFrameInfo.time;
        whistle.confidenceOfLastWhistleDetection = 100;
      }
      // One ear is damaged but I can hear the sound on the other ear:
      else if ((w0 && !theDamageConfigurationHead.audioChannel0Defect && theDamageConfigurationHead.audioChannel1Defect) ||
               (w1 && !theDamageConfigurationHead.audioChannel1Defect && theDamageConfigurationHead.audioChannel0Defect)) {
        whistle.lastTimeWhistleDetected = theFrameInfo.time;
        whistle.confidenceOfLastWhistleDetection = 50;
      }
      // Last (positive) case: Both ears are OK, but I can hear the sound on one ear, only:
      else if (!theDamageConfigurationHead.audioChannel0Defect && !theDamageConfigurationHead.audioChannel1Defect && vol &&
               ((w0 && !w1) || (!w0 && w1))) {
        whistle.lastTimeWhistleDetected = theFrameInfo.time;
        if (theFrameInfo.getTimeSince(lastTimeWhistleDetectedInBothChannels) > timeForOneChannelAcceptance) {
          whistle.confidenceOfLastWhistleDetection = 50;
        } else {
          whistle.confidenceOfLastWhistleDetection = 100;
        }
      }
      // Finally, a completely deaf robot has a negative confidence:
      else if (theDamageConfigurationHead.audioChannel0Defect && theDamageConfigurationHead.audioChannel1Defect) {
        whistle.confidenceOfLastWhistleDetection = -1; // Not a detection but other robots get the information to ignore me
      }
    }

    // Finally, plot the whistle probability of each channel:
    if (!deactivatePlots) {
      DECLARE_PLOT("module:TfLiteWhistleRecognizer:whistleProbabilityChannel0");
      DECLARE_PLOT("module:TfLiteWhistleRecognizer:whistleProbabilityChannel1");
      PLOT("module:TfLiteWhistleRecognizer:whistleProbabilityChannel0", probChannel0);
      PLOT("module:TfLiteWhistleRecognizer:whistleProbabilityChannel1", probChannel1);
      DECLARE_PLOT("module:TfLiteWhistleRecognizer:audioInput60HzChannel0");
      DECLARE_PLOT("module:TfLiteWhistleRecognizer:audioInput60HzChannel1");
      PLOT("module:TfLiteWhistleRecognizer:audioInput60HzChannel0", inputChannel0.back());
      PLOT("module:TfLiteWhistleRecognizer:audioInput60HzChannel1", inputChannel1.back());
      DECLARE_PLOT("module:TfLiteWhistleRecognizer:currentVolume");
      PLOT("module:TfLiteWhistleRecognizer:currentVolume", currentVolume);
    }
    if (whistle.lastTimeWhistleDetected == theFrameInfo.time) {
      OUTPUT_TEXT("Whistle detected");
      whistle.whistleDetected = true;
    }
  }
}

bool TfLiteWhistleRecognizer::detectWhistle(const RingBuffer<short, WHISTLE_BUFF_LEN>& inputChannel,
                                            double& probability,
                                            Whistle& whistle) {
  for (int j = 0; j < WHISTLE_BUFF_LEN; j++) {
    whistleInput8kHz[j] = (double)inputChannel[j];
  }

  for (int j = 0; j < WHISTLE_NUM_FRAMES; j++) {
    mfcc(whistleMFCCs + j * WHISTLE_NUM_MFCCS,
         whistleInput8kHz + j * WHISTLE_FRAME_LEN,
         /* sampling rate */ 8192,
         /* filters */ 80,
         WHISTLE_NUM_MFCCS,
         WHISTLE_FRAME_LEN,
         /* lifter */ 0,
         /* append energy? */ 0,
         /* window */ 2,
         WHISTLE_FRAME_LEN);
  }

  uint8_t* input = interpreter_->typed_input_tensor<uint8_t>(0);
  for (int j = 0; j < WHISTLE_NUM_FRAMES * WHISTLE_NUM_MFCCS; j++) {
    input[j] = (uint8_t)(whistleMFCCs[j] / input_scale + input_zero_point);
  }

  VERIFY(interpreter_->Invoke() == kTfLiteOk);

  uint8_t* output = interpreter_->typed_output_tensor<uint8_t>(0);
  probability = 1.0 - ((double)(output[0] - output_zero_point)) * output_scale;
  if (probability >= whistleThreshold) {
    OUTPUT_TEXT("Whistle has been blown! Probability: " << probability);
    return true;
  }
  return false;
}

float TfLiteWhistleRecognizer::computeCurrentVolume() {
  float volume0 = 0.0;
  float volume1 = 0.0;
  if (!theDamageConfigurationHead.audioChannel0Defect) {
    for (int i = 0; i < inputChannel0.getNumberOfEntries(); i++) {
      float sample = static_cast<float>(inputChannel0.getEntry(i)) / static_cast<float>(std::numeric_limits<short>::max());
      volume0 += std::abs(sample);
    }
  } else {
    OUTPUT_WARNING("audio channel0 defect");
  }
  if (!theDamageConfigurationHead.audioChannel1Defect) {
    for (int i = 0; i < inputChannel1.getNumberOfEntries(); i++) {
      float sample = static_cast<float>(inputChannel1.getEntry(i)) / static_cast<float>(std::numeric_limits<short>::max());
      volume1 += std::abs(sample);
    }
  } else {
    OUTPUT_WARNING("audio channel1 defect");
  }
  // OUTPUT_TEXT("volume0: " << volume0 << ", volume1: " << volume1);
  volume0 /= static_cast<float>(inputChannel0.getNumberOfEntries());
  volume1 /= static_cast<float>(inputChannel1.getNumberOfEntries());
  if (!theDamageConfigurationHead.audioChannel0Defect && !theDamageConfigurationHead.audioChannel1Defect) {
    return (volume0 + volume1) / 2.f;
  } else if (theDamageConfigurationHead.audioChannel0Defect) {
    return volume1;
  } else { // this means if(theDamageConfiguration.audioChannel1Defect)
    return volume0;
  }
}

//#endif
MAKE_MODULE(TfLiteWhistleRecognizer, Modeling)
