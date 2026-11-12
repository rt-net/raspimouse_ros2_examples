#!/usr/bin/env bash

set -eu

cd "$(dirname "$0")"

# C++: ament_uncrustify で整形する
ament_uncrustify --reformat
