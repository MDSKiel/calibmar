
target_sources(calibmar_tests PRIVATE
	pose_suggestion_test.cpp)
	
add_test(NAME pose_suggestion_test COMMAND calibmar_tests)