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

#include <pybind11/chrono.h>
#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include "eyeGazeReader.h"

namespace py = pybind11;
using namespace pybind11::literals;

namespace ark::datatools::mpsIO {

void exportMpsIO(py::module& m) {
  m.doc() = "A pybind11 binding for Aria Research Kit (ARK) mps_io library.";

  py::class_<EyeGaze>(m, "EyeGaze")
      .def_readwrite("gaze_vector", &EyeGaze::gaze_vector)
      .def_readwrite("uncertainty", &EyeGaze::uncertainty);

  m.def("readEyeGaze", &readEyeGaze, "path"_a);
}

} // namespace ark::datatools::mpsIO
