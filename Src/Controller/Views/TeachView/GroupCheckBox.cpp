/**
 * @file GroupCheckBox.cpp
 *
 * Implementation of class GroupCheckBox.
 *
 * This file is subject to the terms of MIT License.
 * A copy of this license is included in LICENSE.txt.
 * (c) 2023 NomadZ team
 */

#include <map>
#include "GroupCheckBox.h"

void GroupCheckBox::nextCheckState() {
  std::map<Qt::CheckState, Qt::CheckState> nextStates = {
    {Qt::Unchecked, Qt::Checked},
    {Qt::PartiallyChecked, Qt::Checked},
    {Qt::Checked, Qt::Unchecked},
  };

  setCheckState(nextStates.at(checkState()));
}
