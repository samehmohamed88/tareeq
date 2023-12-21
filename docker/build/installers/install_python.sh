# Fail on first error.
set -e

cd "$(dirname "${BASH_SOURCE[0]}")"

apt-get -y update && apt-get -y install python3 python3-pip python3-dev python3-venv
