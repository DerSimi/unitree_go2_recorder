git clone https://github.com/shacklettbp/madrona_mjx.git

cd madrona_mjx
git submodule update --init --recursive
mkdir build
cd build
cmake ..
make -j

cd ..
pip install -e .
