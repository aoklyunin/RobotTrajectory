#!/bin/bash

source params.sh





if [ -z "$version" ]; then
    version="0.0.1"
fi

PATH="~/.gem/ruby/2.0.0/bin:$PATH"
#share_dir="/opt/tra/share/${package_name}"
#include_dir="/opt/tra/include/${package_name}"
bin_dir="/opt/tra/path_finding/bin"
test_dir="/opt/tra/path_finding/test"
path_log_dir="/opt/tra/path_finding/log/path"
lib_dir="/opt/tra/path_finding/lib"
config_dir="/opt/tra/path_finding/config"
models_dir="/opt/tra/path_finding/models"
#scripts_dir="/opt/tra/path_finding/scripts"



cwd=`pwd`/../project
echo $cwd
root=$cwd
build=`mktemp -d`
after_install="after-install.sh"
before_remove="before-remove.sh"

cd $root

function cleanup()
{
    echo -e "\033[31m$1\033[0m"

    cd $cwd
    #rm -r $build
    exit 1
}

timestamp=`date +'%s'`
#mkdir -p $build${share_dir}
#mkdir -p $build${include_dir}
mkdir -p $build${path_log_dir}
#mkdir -p $build${scripts_dir}
mkdir -p $build${bin_dir}
mkdir -p $build${lib_dir}
mkdir -p $build${config_dir}
mkdir -p $build${models_dir}
mkdir -p $build${test_dir}



for f in ${bin_files[*]} ; do
    cp $root/$f $build${bin_dir} -R || cleanup "failed to copy file $f"
done

#for f in ${include_files[*]} ; do
#    cp $root/$f $build${include_dir} -R || cleanup "failed to copy file $f"
#done
for f in ${lib_files[*]} ; do
    cp $root/$f $build${lib_dir} -R || cleanup "failed to copy file $f"
done
#for f in ${share_files[*]} ; do
#    cp $root/$f $build${share_dir} -R || cleanup "failed to copy file $f"
#done

for f in ${models_files[*]} ; do
    cp $root/$f $build${models_dir} -R || cleanup "failed to copy file $f"
done

for f in ${config_files[*]} ; do
    cp $root/$f $build${config_dir} -R || cleanup "failed to copy file $f"
done
for f in ${test_files[*]} ; do
    cp $root/$f $build${test_dir} -R || cleanup "failed to copy file $f"
done


cp $root/../deploy/${after_install} $build -R || cleanup "failed to copy file $f"
cp $root/../deploy/${before_remove} $build -R || cleanup "failed to copy file $f"

depends_list=""
for f in ${depends[*]} ; do
    depends_list="${depends_list} --depends $f "
done

cd $build

fpm \
	-s dir \
	-t deb \
	--prefix / \
	--name ${package} \
	-v ${version} \
	--category misc \
	--architecture amd64 \
	--maintainer "${maintainer}" \
	--url "$url" \
	--license $license \
	--description "$description" \
	--vendor "$vendor" \
	--after-install "./${after_install}" \
	--before-remove "./${before_remove}" \
    ${depends_list} \
    . \
    || cleanup "fpm failed"

debfile=`ls *.deb`



cd $cwd

if [[ -e $debfile ]]
then
    cleanup "deb package $debfile already exist (same version?)"
fi

cp -n $build/*.deb .

#cp -R

echo -e "\033[32mCreated package $debfile\033[0m"
