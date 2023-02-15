/**
 * @file ThumbnailProvider.h
 *
 * This file is subject to the terms of the BHuman 2013 License.
 * A copy of this license is included in LICENSE.B-Human.txt.
 * (c) 2022 BHuman and NomadZ team
 *
 * @note The original author is Alexis Tsogias
 */

#pragma once

#include "Core/Module/Module.h"
#include "Representations/Infrastructure/Thumbnail.h"

MODULE(ThumbnailProvider)
REQUIRES(Image)
PROVIDES_WITH_OUTPUT_AND_DRAW(Thumbnail)
DEFINES_PARAMETER(unsigned int, downScales, 3)
END_MODULE

class ThumbnailProvider : public ThumbnailProviderBase {
public:
  ThumbnailProvider();

  void update(Thumbnail& thumbnail);

private:
  void shrinkNxN(const Image& srcImage, Thumbnail::ThumbnailImage& destImage);

  void shrink8x8SSE(const Image& srcImage, Thumbnail::ThumbnailImage& destImage);
  void shrink4x4SSE(const Image& srcImage, Thumbnail::ThumbnailImage& destImage);
};