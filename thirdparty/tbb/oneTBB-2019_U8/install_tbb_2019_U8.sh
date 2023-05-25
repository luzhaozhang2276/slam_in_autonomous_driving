#!/bin/bash
#

MY_LOCAL_DIR="/usr/local"
TBB_ROOT_DIR="${MY_LOCAL_DIR}/tbb-2019_U8"

if [ ! -d ${TBB_ROOT_DIR} ]; then  
  sudo mkdir ${TBB_ROOT_DIR}
else
  sudo rm -rf ${TBB_ROOT_DIR}/*
fi

sudo cp -r include ${TBB_ROOT_DIR}/include
sudo mkdir ${TBB_ROOT_DIR}/lib
sudo cp build/my_tbb_build_release/*so* ${TBB_ROOT_DIR}/lib

if [ -e ${MY_LOCAL_DIR}/include/tbb ]; then  
  sudo rm -rf ${MY_LOCAL_DIR}/include/tbb
fi
if [ -e ${MY_LOCAL_DIR}/lib/libtbb.so ]; then  
  sudo rm -f ${MY_LOCAL_DIR}/lib/libtbb.so
fi
if [ -e ${MY_LOCAL_DIR}/lib/libtbbmalloc.so ]; then  
  sudo rm -f ${MY_LOCAL_DIR}/lib/libtbbmalloc.so
fi
if [ -e ${MY_LOCAL_DIR}/lib/libtbbmalloc_proxy.so ]; then  
  sudo rm -f ${MY_LOCAL_DIR}/lib/libtbbmalloc_proxy.so
fi

sudo ln -s ${TBB_ROOT_DIR}/include/tbb ${MY_LOCAL_DIR}/include/tbb
sudo ln -s ${TBB_ROOT_DIR}/lib/libtbb.so.2 ${MY_LOCAL_DIR}/lib/libtbb.so
sudo ln -s ${TBB_ROOT_DIR}/lib/libtbbmalloc.so.2 ${MY_LOCAL_DIR}/lib/libtbbmalloc.so
sudo ln -s ${TBB_ROOT_DIR}/lib/libtbbmalloc_proxy.so.2 ${MY_LOCAL_DIR}/lib/libtbbmalloc_proxy.so

if [ -z "${LD_LIBRARY_PATH}" ]; then
  echo "export LD_LIBRARY_PATH=${TBB_ROOT_DIR}/lib" >> ~/.zshrc  
else  
  tbb_result=$(echo ${LD_LIBRARY_PATH} | grep "${TBB_ROOT_DIR}/lib")
  if [ -z "${tbb_result}" ]; then    
    echo "export LD_LIBRARY_PATH=${TBB_ROOT_DIR}/lib:${LD_LIBRARY_PATH}" >> ~/.zshrc    
  fi
fi
source ~/.zshrc

