/*
 * libmfcc.h - Header file for libmfcc.c
 * Author - Akarsh Prabhakara
 *
 * MIT License
 * Copyright (c) 2017 Akarsh Prabhakara
 */

#pragma once

#include <cmath>
#include <vector>

namespace nomadz_audio_processing {

  void mfcc(std::vector<double>& mfcc_result,
            const std::vector<double>& data,
            int samplingRate,
            int nfilt,
            int numcep,
            int nfft,
            int ceplifter,
            int appendEnergy,
            int window,
            int numentries);
  void getFilterBankParameters(std::vector<std::vector<double>>& fbank, int nfilt, int samplingRate, int nfft);
  double hztomel(double hz);
  double meltohz(double mel);
  void windowing(std::vector<double>& temp_in, int window);
} // namespace nomadz_audio_processing
