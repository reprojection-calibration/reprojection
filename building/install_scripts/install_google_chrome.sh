#!/bin/bash

set -eoux pipefail

# Kaleido is needed by plotly for image export, which needs google chrome apparently...

apt-get update
apt-get install --no-install-recommends --yes  \
  wget \
  gnupg \
  ca-certificates

wget --quiet -O - https://dl.google.com/linux/linux_signing_key.pub | gpg --dearmor -o /usr/share/keyrings/google-linux-keyring.gpg
echo "deb [arch=amd64 signed-by=/usr/share/keyrings/google-linux-keyring.gpg] http://dl.google.com/linux/chrome/deb/ stable main" > /etc/apt/sources.list.d/google-chrome.list

apt-get update
apt-get install --no-install-recommends --yes \
  google-chrome-stable

rm -rf /var/lib/apt/lists/*
