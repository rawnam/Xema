#!/bin/bash
commit_ts=`git log -1 --format="%ct"`
commit_time=`date -d@$commit_ts +"%Y-%m-%d %H:%M:%S"`
current_time=`date +"%Y-%m-%d %H:%M:%S"`
git_version=`git log -1 --format="%h"`
camera_model=''
sed s/MYVERSION/"XEMA$camera_model\ version: $git_version\ commit: $commit_time\ build: $current_time"/g version.h.tmp > ../firmware/version.h