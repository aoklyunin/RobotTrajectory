#!/bin/bash

source params.sh

last_package=`ls "${package}"* | sort | tail -1`

scp -i ~/.ssh/apt-server ${last_package} apter@apt.tra.ai:/packages
