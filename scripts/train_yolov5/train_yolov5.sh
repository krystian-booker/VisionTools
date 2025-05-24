#!/usr/bin/env bash
set -euo pipefail

if [ ! -d ".venv" ]; then
  echo "⏳ Creating virtual environment..."
  python3 -m venv .venv
fi

source .venv/bin/activate

echo "🔄 Upgrading pip..."
pip install --upgrade pip >/dev/null
if ! pip show ultralytics >/dev/null 2>&1; then
  echo "📥 Installing ultralytics..."
  pip install ultralytics >/dev/null
fi

read -p "Enter the path to your data.yaml file: " DATA_PATH

python train.py --data "$DATA_PATH"
