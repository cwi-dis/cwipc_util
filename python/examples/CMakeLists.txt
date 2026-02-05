cmake_minimum_required(VERSION 3.28.0)

add_test(NAME python_compressfile 
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/compressfile.py ${CMAKE_SOURCE_DIR}/cwipc_util/tests/fixtures/input/pcl_frame1.ply ./pcl_frame1.cwicpc
)
cwipc_python_test(TEST python_compressfile FIXTURES_REQUIRED venv_with_cwipc)
