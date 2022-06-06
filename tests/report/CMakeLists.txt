
target_sources(calibmar_tests PRIVATE
	report_test.cpp)
	
add_test(NAME report_test COMMAND calibmar_tests)