# Fail on first error.
set -e

cd "$(dirname "${BASH_SOURCE[0]}")"

pip3 install torch==2.1.2 torchvision torchaudio
