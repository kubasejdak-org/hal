#!/bin/bash

cmake .. -DPLATFORM=linux -DTOOLCHAIN=clang-9 -DAPP=middleware "${@}"
