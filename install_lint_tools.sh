#!/usr/bin/env bash

# lint.sh と format.sh で使う clang-format と ruff をインストールする
# ament_* のツールは rosdep install でインストールする
set -eu

# root で実行する場合（CI のコンテナなど）は sudo を使わない
if [ "$(id -u)" -eq 0 ]; then
  sudo=""
else
  sudo="sudo"
fi

$sudo apt-get update
$sudo apt-get install -y --no-install-recommends clang-format pipx

# ruff は apt や rosdep で入らないため pipx でバージョンを固定してインストールする
pipx install ruff==0.17.3
