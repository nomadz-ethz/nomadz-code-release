/**
 * @file TfLiteWhistleRecognizer.h
 *
 * Declaration of module that identifies the sound of a whistle
 *
 * This file is subject to the terms of MIT License.
 * A copy of this license is included in LICENSE.txt.
 * (c) 2023 NomadZ team
 */

#pragma once

#include "Core/Module/Module.h"
#include "Core/RingBuffer.h"
#include "Representations/Configuration/DamageConfiguration.h"
#include "Representations/Infrastructure/AudioData.h"
#include "Representations/Infrastructure/FrameInfo.h"
#include "Representations/Infrastructure/GameInfo.h"
#include "Representations/Infrastructure/RobotInfo.h"
#include "Representations/Sensing/FallDownState.h"
#include "Representations/Modeling/Whistle.h"

#include "libmfcc/src/libmfcc.h"

#include "tensorflow/lite/interpreter.h"
#include "tensorflow/lite/model.h"

const int WHISTLE_BUFF_LEN = 8000;
const int WHISTLE_FRAME_LEN = 1000;
const int WHISTLE_NUM_FRAMES = 8;
const int WHISTLE_NUM_MFCCS = 64;
const int WHISTLE_OVERLAP = (WHISTLE_BUFF_LEN - 2000); // make 1/4 of the buffer old data and 3/4 new data (2048 of old data)

MODULE(TfLiteWhistleRecognizer)
REQUIRES(AudioData)
REQUIRES(FrameInfo)
REQUIRES(GameInfo)
REQUIRES(RobotInfo)
REQUIRES(DamageConfiguration)
REQUIRES(DamageConfigurationHead)
REQUIRES(FallDownState)

PROVIDES_WITH_MODIFY_AND_OUTPUT(Whistle)

USES(TeamMateData)
LOADS_PARAMETER(std::string, pathToModel)
LOADS_PARAMETER(float, whistleThreshold)          /**< Minimum correlation for accepting sound as whistle */
LOADS_PARAMETER(float, volumeThreshold)           /**< Minimum sound intensity for accepting sound as whistle */
LOADS_PARAMETER(int, timeForOneChannelAcceptance) /**< After having heard the whistle on both channels, this is the amount of
                                                     time in which one channel is still sufficient */
LOADS_PARAMETER(bool, deactivatePlots)            /**< Unimportant flag */
END_MODULE
/*
 * @class TfLiteWhistleRecognizer
 *
 * Module that identifies the sound of a whistle in audio data
 */
class TfLiteWhistleRecognizer : public TfLiteWhistleRecognizerBase {
public:
  /** Constructor */
  TfLiteWhistleRecognizer();

  /** Destructor */
  ~TfLiteWhistleRecognizer();
  static bool whistleDetected;

private:
  RingBuffer<short, WHISTLE_BUFF_LEN> inputChannel0; /** Audio data from the first channel */
  RingBuffer<short, WHISTLE_BUFF_LEN> inputChannel1; /** Audio data from the second channel */

  double* whistleInput8kHz;
  double* whistleMFCCs;

  int cmpCnt;                                         /**< Number of new audio samples */
  uint8_t lastGameState;                              /**< Keep last game state for checking state transition to SET */
  unsigned int lastTimeWhistleDetectedInBothChannels; /**< As the name says ... */

  unsigned int firstTimeUpright;

  std::unique_ptr<tflite::FlatBufferModel> model_;
  std::unique_ptr<tflite::Interpreter> interpreter_;

  float input_scale, output_scale;
  int input_zero_point, output_zero_point;

  /**
   * Method for recognizing a whistle in one channel
   * @param inputChannel The incoming audio data
   * @param correlation Returns the correlation of the incoming data with the preconfigured reference whistle
   * @return true, if a whistle has been recognized, false otherwise
   */
  bool detectWhistle(const RingBuffer<short, WHISTLE_BUFF_LEN>& inputChannel,
                     double& probability,
                     Whistle& whistle,
                     int channel_num);

  /**
   * Determines the current average sound level
   */
  float computeCurrentVolume();

  /**
   * The method that detects the whistle
   * @param Whistle The identified whistle
   */
  void update(Whistle& whistle);
};
