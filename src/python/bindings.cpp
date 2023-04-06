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

#include "AriaDataProviderPyBind.h"
#include "AriaPlayersPyBind.h"
#include "DeviceModelPyBind.h"
#include "MpsIOPyBind.h"

namespace py = pybind11;
using namespace ark::datatools;

PYBIND11_MODULE(projectaria, m) {
  py::module tools = m.def_submodule("tools");

  py::module dataprovider = tools.def_submodule("dataprovider");
  dataprovider::exportPlayers(dataprovider);
  dataprovider::exportDataProvider(dataprovider);

  py::module sensors = tools.def_submodule("sensors");
  sensors::exportSensors(sensors);

  py::module mpsIO = tools.def_submodule("mps_io");
  mpsIO::exportMpsIO(mpsIO);
}
