
option(test "Build all tests." OFF) # Turn on by giving -Dtest=ON

enable_testing()
find_package(GTest)

file(GLOB source *.cpp)
add_executable(tests ${source})
target_link_libraries(tests pclem GTest::GTest GTest::Main)

add_test(AllTests tests)
