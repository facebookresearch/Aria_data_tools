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

#include <models/DeviceModel.h>
#include <sophus/se3.hpp>
#include <utility/VrsUtils.h>
#include <vrs/RecordFileReader.h>

#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

namespace py = pybind11;
using namespace pybind11::literals;

namespace ark {
namespace datatools {
namespace sensors {

void exportSensors(py::module& m) {
  m.doc() = "A pybind11 binding for Aria Research Kit (ARK) camera model and calibration APIs.";
  m.def(
      "test", [](const int a, const int b) { return a + b; }, "a"_a, "b"_a);
  py::enum_<CameraProjectionModel::ModelType>(m, "CameraModelType")
      .value("KannalaBrandtK3", CameraProjectionModel::ModelType::KannalaBrandtK3)
      .value("Fisheye624", CameraProjectionModel::ModelType::Fisheye624)
      .export_values();

  py::class_<CameraProjectionModel>(m, "CameraProjectionModel")
      .def(py::init<>())
      .def_readwrite("modelName", &CameraProjectionModel::modelName)
      .def_readwrite("projectionParams", &CameraProjectionModel::projectionParams)
      .def("project", &CameraProjectionModel::project, "p"_a)
      .def("unproject", &CameraProjectionModel::unproject, "uv"_a);

  py::class_<Sophus::SE3d>(m, "SE3d")
      .def(py::init<>())
      .def("rotationMatrix", [](const Sophus::SE3d& self) { return self.rotationMatrix(); })
      .def("translation", [](const Sophus::SE3d& self) { return self.translation(); });

  py::class_<CameraCalibration>(m, "CameraCalibration")
      .def(py::init<>())
      .def_readwrite("label", &CameraCalibration::label)
      .def_readwrite("projectionModel", &CameraCalibration::projectionModel)
      .def_readonly("T_Device_Camera", &CameraCalibration::T_Device_Camera);

  py::class_<LinearRectificationModel>(m, "LinearRectificationModel")
      .def(py::init<>())
      .def_readwrite("rectificationMatrix", &LinearRectificationModel::rectificationMatrix)
      .def_readwrite("bias", &LinearRectificationModel::bias)
      .def("rectify", &LinearRectificationModel::rectify, "v"_a);

  py::class_<ImuCalibration>(m, "ImuCalibration")
      .def(py::init<>())
      .def_readwrite("label", &ImuCalibration::label)
      .def_readwrite("accel", &ImuCalibration::accel)
      .def_readwrite("gyro", &ImuCalibration::gyro)
      .def_readonly("T_Device_Imu", &ImuCalibration::T_Device_Imu);

  py::class_<DeviceModel>(m, "DeviceModel")
      .def(py::init<>())
      .def_static(
          "fromJson", static_cast<DeviceModel (*)(const std::string&)>(&DeviceModel::fromJson))
      .def(
          "getCameraCalib",
          [](const DeviceModel& self, const std::string& label) {
            const auto ret = self.getCameraCalib(label);
            if (ret.has_value()) {
              return py::cast(ret.value());
            }
            return py::object(py::cast(nullptr));
          })
      .def(
          "getImuCalib",
          [](const DeviceModel& self, const std::string& label) {
            const auto ret = self.getImuCalib(label);
            if (ret.has_value()) {
              return py::cast(ret.value());
            }
            return py::object(py::cast(nullptr));
          })
      .def("getCameraLabels", &DeviceModel::getCameraLabels)
      .def("getImuLabels", &DeviceModel::getImuLabels)
      .def(
          "transform",
          &DeviceModel::transform,
          "p_source"_a,
          "sourceSensorLabel"_a,
          "destSensorLabel"_a);

  // TODO: This is a temporary binding of vrs::RecordFileReader to allow users to
  // extract calibration string from a VRS file. It should be replaced with a more
  // complete binding of vrs library.
  py::class_<vrs::RecordFileReader>(m, "RecordFileReader")
      .def(py::init<>())
      .def(
          "openFile",
          [](vrs::RecordFileReader& self, const std::string& filePath) {
            return self.openFile(filePath);
          },
          "filePath"_a);
  m.def("getCalibrationFromVrsFile", &getCalibrationFromVrsFile, "reader"_a);
}
} // namespace sensors
} // namespace datatools
} // namespace ark
