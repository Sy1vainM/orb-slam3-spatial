set -eax

SELF=$(SELF=$(dirname "$0") && bash -c "cd \"$SELF\" && pwd")

echo "Configuring and building Thirdparty/DBoW2 ..."

mkdir -p $SELF/Thirdparty/DBoW2/build
cd $SELF/Thirdparty/DBoW2/build
cmake .. -DCMAKE_BUILD_TYPE=Release
cmake --build . -- -j

echo "Configuring and building Thirdparty/g2o ..."

mkdir -p $SELF/Thirdparty/g2o/build
cd $SELF/Thirdparty/g2o/build
cmake .. -DCMAKE_BUILD_TYPE=Release
cmake --build . -- -j

echo "Configuring and building Thirdparty/Sophus ..."

mkdir -p $SELF/Thirdparty/Sophus/build
cd $SELF/Thirdparty/Sophus/build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j

echo "Configure and build Pangolin"
if [! -d $SELF/Thirdparty/Pangolin ]; then
  cd $SELF/Thirdparty
  # Get Pangolin code
  git clone --recursive https://github.com/stevenlovegrove/Pangolin.git
fi
cd $SELF/Thirdparty/Pangolin
# Configure and build
cmake -B build
cmake --build build -- -j

echo "Uncompress vocabulary ..."

if [! -f $SELF/Vocabulary/ORBvoc.txt ]; then
  cd $SELF/Vocabulary
  tar -xf ORBvoc.txt.tar.gz
fi

echo "Configuring and building ORB_SLAM3 ..."

if [! -d $SELF/build ]; then
  mkdir -p $SELF/build
fi
cd $SELF/build
cmake .. -DCMAKE_BUILD_TYPE=Release
cmake --build .
