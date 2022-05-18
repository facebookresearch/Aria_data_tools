/*
 * Copyright (c) Meta Platforms, Inc. and affiliates.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#pragma once

#include <models/fisheye.h>
#include <models/kannala_brandt.h>
#include <models/pinhole.h>

#define CALL_PREPROC_MACRO_FOR_EACH_PROJECTOR(macro) \
  macro(PinholeProjector);                           \
  macro(Fisheye624Projector);                        \
  macro(KannalaBrandt3Projector)

#define CALL_PREPROC_MACRO_FOR_EACH_PROJECTOR_WITH_NAME(macro) \
  macro(PinholeProjector, Pinhole);                            \
  macro(Fisheye624Projector, F624);                            \
  macro(KannalaBrandt3Projector, KB3)
