set -e
cd "$(git rev-parse --show-toplevel)"

if [ -d "cmake-build-release/test" ]; then
    cd "cmake-build-release/test"
else
    ./script/build.sh
    cd "cmake-build-release/test"
fi

binaries=(
    "test_jps_embeddings"
    "test_jps_octile"
)

for binary in "${binaries[@]}"; do
    if [ -f "./$binary" ]; then
        ./"$binary" 2>&1 | tee "$binary.txt"
    fi
done
