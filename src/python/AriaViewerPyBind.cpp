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

#include <data_provider/AriaDataProvider.h>
#include <visualization/AriaViewer.h>

#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>

namespace py = pybind11;

namespace ark {
namespace datatools {
namespace visualization {

void exportVisualization(py::module& m) {
  m.doc() = "A pybind11 binding for Aria Data Tools visualization tooling.";
  py::class_<AriaViewer>(m, "AriaViewer")
      .def(py::init<datatools::dataprovider::AriaDataProvider*, int, int>())
      .def("run", &AriaViewer::run, py::call_guard<py::gil_scoped_release>())
      .def("isPlaying", &AriaViewer::isPlaying)
      .def(
          "setDataChanged",
          &AriaViewer::setDataChanged,
          py::arg("setDataChanged"),
          py::arg("streamId"))
      .def("readData", &AriaViewer::readData, py::arg("currentTimestampSec"))
      .def("initDataStreams", &AriaViewer::initDataStreams)
      .def("getPlaybackSpeedFactor", &AriaViewer::getPlaybackSpeedFactor);
}

} // namespace visualization
} // namespace datatools
} // namespace ark
