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

#include <data_provider/AriaVrsDataProvider.h>

#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>

namespace py = pybind11;

namespace ark {
namespace datatools {
namespace dataprovider {

void exportFolderDataProvider(py::module& m) {
  py::class_<AriaFolderDataProvider, AriaDataProvider>(m, "AriaFolderDataProvider")
      .def(py::init<>())
      // AriaDataProvider override functions
      .def("open", &AriaFolderDataProvider::open, py::arg("folderPath"), py::arg("posePath"))
      .def("setStreamPlayer", &AriaFolderDataProvider::setStreamPlayer, py::arg("streamId"))
      .def(
          "tryFetchNextData",
          &AriaFolderDataProvider::tryFetchNextData,
          py::arg("streamId"),
          py::arg("currentTimestampSec"))
      .def(
          "getImageBuffer",
          &AriaFolderDataProvider::getImageBuffer,
          py::return_value_policy::reference,
          py::arg("streamId"))
      .def("getImageWidth", &AriaFolderDataProvider::getImageWidth, py::arg("streamId"))
      .def("getImageHeight", &AriaFolderDataProvider::getImageHeight, py::arg("streamId"))
      .def("getFastestNominalRateHz", &AriaFolderDataProvider::getFastestNominalRateHz)
      .def(
          "getFirstTimestampSec",
          py::overload_cast<>(&AriaFolderDataProvider::getFirstTimestampSec))
      .def("getPose", &AriaFolderDataProvider::getPose)
      .def("atLastRecords", &AriaFolderDataProvider::atLastRecords)
      .def("loadDeviceModel", &AriaFolderDataProvider::loadDeviceModel)
      .def(
          "getDeviceModel",
          &AriaFolderDataProvider::getDeviceModel,
          py::return_value_policy::reference)
      .def("streamExistsInSource", &AriaDataProvider::streamExistsInSource, py::arg("streamId"));
}

} // namespace dataprovider
} // namespace datatools
} // namespace ark
