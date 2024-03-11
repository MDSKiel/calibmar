
target_sources(calibmar_tests PRIVATE
	chessboard_extractor_test.cpp
	arucoboard_extractor_test.cpp)
	
add_test(NAME chessboard_extractor_test COMMAND calibmar_tests)
add_test(NAME aruocboard_extractor_test COMMAND calibmar_tests)