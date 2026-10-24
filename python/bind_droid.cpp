#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include "franky/motion/droid_joint_impedance_motion.hpp"
namespace py=pybind11;
using namespace franky;
void bind_droid(py::module_ &m) {
  py::class_<DroidJointLimits>(m,"DroidJointLimits")
    .def(py::init<>())
    .def_readwrite("lower",&DroidJointLimits::lower).def_readwrite("upper",&DroidJointLimits::upper)
    .def_readwrite("velocity",&DroidJointLimits::velocity).def_readwrite("command_torque",&DroidJointLimits::command_torque)
    .def_readwrite("publisher_timeout_ns",&DroidJointLimits::publisher_timeout_ns)
    .def_readwrite("callback_gap_ns",&DroidJointLimits::callback_gap_ns);
  py::class_<DroidConsumedReference>(m,"DroidConsumedReference")
    .def_readonly("row_sequence",&DroidConsumedReference::row_sequence)
    .def_readonly("publication_sequence",&DroidConsumedReference::publication_sequence)
    .def_readonly("tick",&DroidConsumedReference::tick)
    .def_readonly("monotonic_ns",&DroidConsumedReference::monotonic_ns);
  py::class_<DroidJointSample>(m,"DroidJointSample")
    .def_readonly("tick",&DroidJointSample::tick).def_readonly("monotonic_ns",&DroidJointSample::monotonic_ns)
    .def_readonly("publication_sequence",&DroidJointSample::publication_sequence)
    .def_readonly("consumed",&DroidJointSample::consumed)
    .def_readonly("q",&DroidJointSample::q).def_readonly("dq",&DroidJointSample::dq)
    .def_readonly("reference",&DroidJointSample::reference).def_readonly("raw",&DroidJointSample::raw)
    .def_readonly("tracking_raw",&DroidJointSample::tracking_raw)
    .def_readonly("outgoing",&DroidJointSample::outgoing)
    .def_readonly("fault",&DroidJointSample::fault).def_readonly("stop_fault",&DroidJointSample::stop_fault)
    .def_readonly("dropped",&DroidJointSample::dropped)
    .def_readonly("stopping",&DroidJointSample::stopping).def_readonly("stop_finished",&DroidJointSample::stop_finished)
    .def_readonly("stop_at_rest",&DroidJointSample::stop_at_rest);
  py::class_<DroidJointImpedanceMotion,JointImpedanceBase,std::shared_ptr<DroidJointImpedanceMotion>>(m,"DroidJointImpedanceMotion")
    .def(py::init<const JointImpedanceParams &,const DroidJointLimits &,const TorqueStopParams &,size_t,double>(),
         py::arg("params"),py::arg("limits"),py::arg("stop"),py::arg("log_capacity"),py::arg("gains_time_constant"))
    .def_property_readonly("params",[](const DroidJointImpedanceMotion &motion) {return motion.params();})
    .def("publish",&DroidJointImpedanceMotion::publish,py::arg("q"),py::arg("row_sequence"),py::arg("expires_ns"))
    .def("request_stop",&DroidJointImpedanceMotion::requestStop)
    .def("status",&DroidJointImpedanceMotion::status)
    .def("drain",&DroidJointImpedanceMotion::drain,py::call_guard<py::gil_scoped_release>());
}
