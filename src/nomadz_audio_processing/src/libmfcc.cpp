
#include <cmath>
#include <vector>
#include <cstring>

#include <fftw3.h>

#include "nomadz_audio_processing/libmfcc.hpp"

const double DBL_EPSILON = 2.2204460492503131e-16;
const double PI = 3.14159265358979323846264338327;

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
            int numentries) {
    // Get frame data into temp_in and pad with zeros if numentries < nfft
    std::vector<double> temp_in(nfft, 0.0);
    std::memcpy(temp_in.data(), data.data(), sizeof(double) * numentries);

    // Windowing
    if (window != 0) {
      windowing(temp_in, window);
    }

    // Run FFT
    std::vector<fftw_complex> temp_out(nfft / 2 + 1);
    fftw_plan p =
      fftw_plan_dft_r2c_1d(nfft, temp_in.data(), reinterpret_cast<fftw_complex*>(temp_out.data()), FFTW_ESTIMATE);
    fftw_execute(p);
    fftw_destroy_plan(p);

    // Compute power spectra
    std::vector<double> powspectrum(nfft / 2 + 1);
    for (int i = 0; i < nfft / 2 + 1; ++i) {
      powspectrum[i] = (1.0 / nfft) * (pow(temp_out[i][0], 2) + pow(temp_out[i][1], 2));
    }
    // Compute the filterbank parameters
    std::vector<std::vector<double>> fbank(nfilt, std::vector<double>(nfft / 2 + 1));
    getFilterBankParameters(fbank, nfilt, samplingRate, nfft);

    double specenergy = 0.0;
    for (double ps : powspectrum) {
      specenergy += ps;
    }

    if (specenergy <= 0.0) {
      specenergy = DBL_EPSILON;
    }

    // Get filter bank output
    std::vector<double> feat(nfilt);
    for (int l = 0; l < nfilt; ++l) {
      feat[l] = 0.0;
      for (int k = 0; k < nfft / 2 + 1; ++k) {
        feat[l] += powspectrum[k] * fbank[l][k];
      }

      if (feat[l] > 0.0) {
        feat[l] = log(feat[l]);
      } else {
        feat[l] = DBL_EPSILON;
      }
    }

    // Resize mfcc_result to the appropriate size
    mfcc_result.resize(numcep);

    for (int i = 0; i < numcep; ++i) {
      // DCT - II of filter bank output
      mfcc_result[i] = 0.0;
      for (int j = 0; j < nfilt; ++j) {
        mfcc_result[i] += feat[j] * cos((i * PI / nfilt) * (j + 0.5));
      }

      // Orthogonalization of DCT output
      if (i == 0) {
        mfcc_result[i] *= sqrt(1.0 / nfilt);
      } else {
        mfcc_result[i] *= sqrt(2.0 / nfilt);
      }

      // Ceplifter
      if (ceplifter != 0) {
        mfcc_result[i] *= 1.0 + (ceplifter / 2.0) * sin(PI * i / ceplifter);
      }
    }

    // Append Energy
    if (appendEnergy == 1) {
      mfcc_result[0] = log(specenergy);
    }
  }

  void getFilterBankParameters(std::vector<std::vector<double>>& fbank, int nfilt, int samplingRate, int nfft) {
    double lowmel = hztomel(0.0);
    double highmel = hztomel(samplingRate / 2.0);

    // Generate nfilt center frequencies linearly spaced in the mel scale
    std::vector<double> bin(nfilt + 2);
    for (int i = 0; i <= nfilt + 1; ++i) {
      bin[i] = floor(meltohz(i * (highmel - lowmel) / (nfilt + 1) + lowmel) * (nfft + 1) / samplingRate);
    }

    // Triangular Filter Banks
    for (int i = 0; i < nfilt; ++i) {
      std::fill(fbank[i].begin(), fbank[i].end(), 0.0);
      for (int j = static_cast<int>(bin[i]); j < static_cast<int>(bin[i + 1]); ++j) {
        fbank[i][j] = (j - bin[i]) / (bin[i + 1] - bin[i]);
      }
      for (int j = static_cast<int>(bin[i + 1]); j < static_cast<int>(bin[i + 2]); ++j) {
        fbank[i][j] = (bin[i + 2] - j) / (bin[i + 2] - bin[i + 1]);
      }
    }
  }

  double hztomel(double hz) {
    return 2595 * log10(1 + hz / 700.0);
  }

  double meltohz(double mel) {
    return 700 * (pow(10, mel / 2595.0) - 1);
  }

  void windowing(std::vector<double>& temp_in, int window) {
    // Apply respective window before FFT
    for (int i = 0; i < static_cast<int>(temp_in.size()); ++i) {
      if (window == 1) {
        temp_in[i] *= (0.5 - 0.5 * cos(2 * PI * i / (static_cast<double>(temp_in.size()) - 1)));

      } else if (window == 2) {
        temp_in[i] *= (0.54 - 0.46 * cos(2 * PI * i / (static_cast<double>(temp_in.size()) - 1)));
      }
    }
  }
} // namespace nomadz_audio_processing
