#!/usr/bin/env bash

set -eu

cd "$(dirname "$0")"

# C/C++: .clang-format の設定で整形する
mapfile -t cpp_files < <(git ls-files '*.c' '*.h' '*.cpp' '*.hpp')
clang-format -i "${cpp_files[@]}"

# Python: ruff で整形する（pyproject.toml の設定）
ruff check --fix .
ruff format .
