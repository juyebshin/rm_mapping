# cd thirdparty/libelas
# rm -rf build
# mkdir build
# cd build
# cmake .. -DCMAKE_BUILD_TYPE=Release
# make -j$(($(nproc) - 1))

# cd ../../../

mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j$(($(nproc) - 1))
