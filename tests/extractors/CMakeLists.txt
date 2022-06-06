
target_sources(calibmar_tests PRIVATE
	chessboard_extractor_test.cpp)
	
add_test(NAME chessboard_extractor_test COMMAND calibmar_tests)