
maintainer='Alexey Klyunin <aleksey.klunin@tra.ai>'
package_name='path-finding'
description='Tools for path planning'

bin_files=(\
    ../build/project/visualization/pathFindingVisDemoScene2
    ../build/project/visualization/pathFindingVis4Robots
    ../build/project/path_finding_client/pathFindingClient
    ../build/project/scene_editor/pathFindingSceneEditor
    ../build/project/generator/generator
    )

test_files=(\
    ../build/project/visualization/testVisualization
)

lib_files=(\
    ../build/project/solid3/lib/libsolid3.so
    ../build/project/solid3/lib/libsolid3.so.3.5.8
    )
include_files=(\
    )
share_files=(\
    collider/
    misc/
    path_finding/
    scene_description/
    scene_wrapper/
    solid3/
    urdf_reader/
    visualization/
    ../README.md
    )
models_files=(\
    ../models/kuka_six
    ../models/kuka_ten
    ../models/primitives
    )

config_files=(\
    ../config_deploy/urdf
    ../config_deploy/murdf
    ../config_deploy/paths
    ../config_deploy/reports
    )


depends=(
    libeigen3-dev
    libtinyxml-dev
    libjsoncpp-dev
    freeglut3-dev
    libboost-all-dev
    libboost-thread-dev
    libboost-date-time-dev
    libboost-log-dev
    liborocos-kdl-dev
    libtinyxml-dev
    liburdfdom-dev
    libtbb-dev
    libtbb2
    qt5-default
    )

#--------------------------------------------------
package_prefix='tra-'
vendor='TRA LLC'
license='TODO'
package=${package_prefix}${package_name}
url="http://gitlab.tra.uk/Kinematics/${package_name}"
