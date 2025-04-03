$ErrorActionPreference = "Stop"

$repoRoot = (git rev-parse --show-toplevel).Trim()
Set-Location $repoRoot

if (Test-Path "cmake-build-release\test" -PathType Container) {
    Set-Location "cmake-build-release\test"
} else {
    & ".\script\build.ps1"
    Set-Location "cmake-build-release\test"
}

$binaries = @(
    "test_jps_embeddings"
    "test_jps_octile"
)

foreach ($binary in $binaries) {
    if (Test-Path ".\$binary" -PathType Leaf) {
        & ".\$binary" 2>&1 | Tee-Object -FilePath "$binary.txt"
    }
}
