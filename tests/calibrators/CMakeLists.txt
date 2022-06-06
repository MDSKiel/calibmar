
target_sources(calibmar_tests PRIVATE
	housing_calibrator_test.cpp
	calibrator_test.cpp)

add_test(NAME calibrator_test COMMAND calibmar_tests)
add_test(NAME housing_calibrator_test COMMAND calibmar_tests)