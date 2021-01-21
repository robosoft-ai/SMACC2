sudo apt-get install xterm

cd dependencies
vcs import < ../submodules/navigation2/tools/underlay.repos 
cd ..
rosdep install -q -y       --from-paths src       --skip-keys " slam_toolbox ignore-src 
