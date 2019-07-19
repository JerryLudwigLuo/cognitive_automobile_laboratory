// It is very important that this is the first header in every file.
// This includes some modifications to the boost::python headers that
// make them work with memory-aligned Eigen types.
// For more usage examples, look at
// https://github.com/ethz-asl/programming_guidelines/wiki/Adding-python-bindings-to-your-cpp-catkin-package
#include <boost/python.hpp>
#include <boost/python/suite/indexing/vector_indexing_suite.hpp>

// Now bring in the headers of functions and classes you are wrapping.
#include "local_geographic_cs.hpp"

// The module name here *must* match the name of the python project. You can use the PYTHON_API_MODULE_NAME definition.
BOOST_PYTHON_MODULE(PYTHON_API_MODULE_NAME) {
    // This using statement is just for convenience
    using namespace boost::python;
    using namespace gnss;

    void(LocalGeographicCS::*ll2xyOVERLOAD)(double, double, double&, double&)const= &LocalGeographicCS::ll2xy;
    void(LocalGeographicCS::*xy2llOVERLOAD)(double, double, double&, double&)const= &LocalGeographicCS::xy2ll;

    std::pair<double, double>(LocalGeographicCS::*ll2xyOVERLOADPAIR)(double, double)const= &LocalGeographicCS::ll2xy;
    std::pair<double, double>(LocalGeographicCS::*xy2llOVERLOADPAIR)(double, double)const= &LocalGeographicCS::xy2ll;

    class_<std::pair<double, double> >("DoublePair")
        .def_readwrite("first", &std::pair<double, double>::first)
        .def_readwrite("second", &std::pair<double, double>::second);

    class_<LocalGeographicCS>("LocalGeographicCS", init<double, double>("LocalGeographicCS(data)"))
        .def("ll2xy", ll2xyOVERLOAD)
        .def("ll2xy", ll2xyOVERLOADPAIR)
        .def("xy2ll", xy2llOVERLOAD)
        .def("xy2ll", xy2llOVERLOADPAIR);
}
